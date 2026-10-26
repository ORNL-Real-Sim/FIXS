"""port_slots.py - the port block a co-sim run uses, and its remapped scenario yaml.

Slot 0 is the scenario yaml as written. Slot k >= 1 moves every port the stack
binds into the block at SLOT_BASE + SLOT_STRIDE * k:

    +0  SUMO TraCI           +40     TrafficLayer bridge
    +20 XIL (tcp)            +50..52 CARLA RPC, streaming, secondary
    +30 app subscribers      +60     Traffic Manager

and writes the result to a copy in the run's dir; the source yaml is never edited.
A UDP dyno (XilSetup.Transport: udp) is an endpoint outside the stack and keeps its
port. Relative output paths (DataLogPath, ViewSetup Record) get the run id as their
last dir, so two runs of one yaml do not write the same files.
"""
import os
import re

SLOT_BASE = 20000
SLOT_STRIDE = 100
MAX_SLOT = 50
OFFSETS = {"traci": 0, "xil": 20, "app": 30, "bridge": 40, "carla": 50, "tm": 60}
_GROUP_WIDTH = 10

DEFAULT_TRACI = 1337
DEFAULT_BRIDGE = 4440
DEFAULT_CARLA = 2000
DEFAULT_TM = 8000


class SlotError(Exception):
    """The yaml cannot be moved to a slot without leaving a port behind."""


def block_base(k):
    return SLOT_BASE + SLOT_STRIDE * int(k)


def _load(path):
    import yaml
    with open(path, encoding="utf-8-sig") as f:
        return yaml.safe_load(f) or {}


def _ports_of(value):
    if isinstance(value, bool) or value is None:
        return []
    if isinstance(value, int):
        return [value]
    if isinstance(value, (list, tuple)):
        return [p for p in value if isinstance(p, int) and not isinstance(p, bool)]
    return []


def _subscription_ports(section):
    """Every `port` listed by the subscriptions of one yaml section, in order."""
    out = []
    for value in (section or {}).values():
        if isinstance(value, list):
            for sub in value:
                if isinstance(sub, dict):
                    out += [p for p in _ports_of(sub.get("port")) if p not in out]
    return out


def discover(path=None, doc=None):
    """The ports a scenario yaml makes the stack bind, by role."""
    doc = _load(path) if doc is None else doc
    sim = doc.get("SimulationSetup") or {}
    carla = doc.get("CarlaSetup")
    xil = doc.get("XilSetup") or {}
    found = {"traci": int(sim.get("TrafficSimulatorPort") or DEFAULT_TRACI),
             "has_carla": carla is not None,
             "bridge": None, "carla": None, "tm": None}
    if carla is not None:
        carla = carla or {}
        found["bridge"] = int(carla.get("CarlaClientPort") or DEFAULT_BRIDGE)
        found["carla"] = int(carla.get("CarlaServerPort") or DEFAULT_CARLA)
        found["tm"] = int(carla.get("TrafficManagerPort") or DEFAULT_TM)
    taken = {found[k] for k in ("traci", "bridge", "carla", "tm") if found[k]}
    app = [p for p in _subscription_ports(doc.get("ApplicationSetup"))
           + _ports_of(sim.get("WarmUpServePorts")) if p not in taken]
    found["app"] = list(dict.fromkeys(app))
    found["xil_udp"] = str(xil.get("Transport") or "").strip().lower() == "udp"
    xil_ports = [] if found["xil_udp"] else _subscription_ports(xil)
    found["xil"] = [p for p in dict.fromkeys(xil_ports)
                    if p not in taken and p not in found["app"]]
    return found


def port_map(found, k, keep_carla=False):
    """{old port: new port} for slot k (identity for slot 0)."""
    if int(k) == 0:
        return {}
    base = block_base(k)
    mapping = {found["traci"]: base + OFFSETS["traci"]}
    for role in ("bridge", "tm"):
        if found.get(role):
            mapping[found[role]] = base + OFFSETS[role]
    if found.get("carla") and not keep_carla:
        mapping[found["carla"]] = base + OFFSETS["carla"]
    for role in ("app", "xil"):
        ports = found.get(role) or []
        if len(ports) > _GROUP_WIDTH:
            raise SlotError(f"{len(ports)} {role} ports do not fit a slot's "
                            f"{_GROUP_WIDTH}-port group")
        for i, p in enumerate(ports):
            mapping.setdefault(p, base + OFFSETS[role] + i)
    return mapping


def bound_ports(found, k, keep_carla=False, with_carla=True):
    """[(label, port)] the run on slot k will bind, for the socket-table check."""
    m = port_map(found, k, keep_carla)

    def at(p):
        return m.get(p, p)
    out = [("SUMO TraCI", at(found["traci"]))]
    out += [("app", at(p)) for p in found.get("app") or []]
    out += [("XIL", at(p)) for p in found.get("xil") or []]
    if found.get("bridge"):
        out.append(("TrafficLayer bridge", at(found["bridge"])))
    if found.get("tm"):
        out.append(("Traffic Manager", at(found["tm"])))
    if with_carla and found.get("carla") and not keep_carla:
        c = at(found["carla"])
        out += [("CARLA RPC", c), ("CARLA streaming", c + 1), ("CARLA secondary", c + 2)]
    return out


def per_run_path(path, run_id):
    """`path` with the run id as its last dir; absolute paths are left alone."""
    if not path or os.path.isabs(path):
        return path
    head, tail = os.path.split(path.replace("\\", "/"))
    return "/".join(p for p in (head, run_id, tail) if p)


_SECTION = re.compile(r"^([A-Za-z_][\w]*)\s*:")
_SCALAR_PORT = re.compile(r"^(\s*)(TrafficSimulatorPort|CarlaServerPort|CarlaClientPort|"
                          r"TrafficManagerPort)(\s*:\s*)(\d+)(.*)$")
_PORT_LIST = re.compile(r"^(\s*(?:-\s*)?(?:port|WarmUpServePorts)\s*:\s*)(\[[^\]]*\]|\d+)(.*)$")
_PATH_KEY = re.compile(r"^(\s*(?:-\s*)?(DataLogPath|Record|SumoConfigFile)\s*:\s*)"
                       r"(['\"]?)([^'\"#\r\n]*?)\3(\s*(?:#.*)?)$")


def _remap_numbers(text, mapping):
    return re.sub(r"\d+", lambda m: str(mapping.get(int(m.group()), int(m.group()))), text)


def _rewrite_lines(lines, mapping, run_id, src_dir, tm_port):
    section, out, tm_seen = None, [], False
    carla_at = None
    for line in lines:
        eol = "\n" if line.endswith("\n") else ""
        m = _SECTION.match(line)
        if m:
            section = m.group(1)
            if section == "CarlaSetup":
                carla_at = len(out)
            out.append(line)
            continue
        m = _SCALAR_PORT.match(line)
        if m and section in ("SimulationSetup", "CarlaSetup"):
            if m.group(2) == "TrafficManagerPort":
                tm_seen = True
            new = mapping.get(int(m.group(4)), int(m.group(4)))
            out.append(f"{m.group(1)}{m.group(2)}{m.group(3)}{new}{m.group(5)}{eol}")
            continue
        m = _PORT_LIST.match(line)
        if m and section in ("ApplicationSetup", "XilSetup", "SimulationSetup"):
            out.append(f"{m.group(1)}{_remap_numbers(m.group(2), mapping)}{m.group(3)}{eol}")
            continue
        m = _PATH_KEY.match(line)
        if m and m.group(4).strip():
            key, value = m.group(2), m.group(4).strip()
            new = value
            if key == "SumoConfigFile" and section == "SumoSetup" and not os.path.isabs(value):
                new = os.path.normpath(os.path.join(src_dir, value)).replace("\\", "/")
            elif ((key == "DataLogPath" and section == "DataLogSetup" and value != "auto")
                  or (key == "Record" and section == "ViewSetup"
                      and value.lower() not in ("false", "true", "null", "~"))):
                new = per_run_path(value, run_id)
            if new != value:
                q = m.group(3) or "'"
                out.append(f"{m.group(1)}{q}{new}{q}{m.group(5)}{eol}")
                continue
        out.append(line)
    if tm_port is not None and not tm_seen and carla_at is not None:
        indent = "  "
        for line in out[carla_at + 1:]:
            if line.strip() and line[0].isspace() and not line.lstrip().startswith("#"):
                indent = line[:len(line) - len(line.lstrip())]
                break
        out.insert(carla_at + 1, f"{indent}TrafficManagerPort: {tm_port}\n")
    return out


def write_slot_yaml(src, dst, k, run_id, keep_carla=False):
    """Write src remapped to slot k at dst. Returns {old: new}. Raises SlotError if a
    port the stack binds would be left outside the slot."""
    doc = _load(src)
    found = discover(doc=doc)
    mapping = port_map(found, k, keep_carla)
    tm_new = mapping.get(found["tm"]) if found.get("tm") else None
    with open(src, encoding="utf-8-sig") as f:
        lines = f.readlines()
    out = _rewrite_lines(lines, mapping, run_id, os.path.dirname(os.path.abspath(src)),
                         tm_new)
    text = "".join(out)
    import yaml
    try:
        after = discover(doc=yaml.safe_load(text) or {})
    except yaml.YAMLError as exc:
        raise SlotError(f"the remapped yaml does not parse: {exc}") from exc
    want = {role: mapping.get(found[role], found[role])
            for role in ("traci", "bridge", "carla", "tm") if found.get(role)}
    for role, port in want.items():
        if after.get(role) != port:
            raise SlotError(f"{role} port is {after.get(role)} after the rewrite, "
                            f"expected {port}")
    for role in ("app", "xil"):
        expect = [mapping.get(p, p) for p in found.get(role) or []]
        if sorted(after.get(role) or []) != sorted(expect):
            raise SlotError(f"{role} ports are {after.get(role)} after the rewrite, "
                            f"expected {expect} (a port list this rewrite cannot read?)")
    os.makedirs(os.path.dirname(os.path.abspath(dst)), exist_ok=True)
    tmp = f"{dst}.{os.getpid()}.tmp"
    with open(tmp, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    os.replace(tmp, dst)
    return mapping
