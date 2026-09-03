"""
peer.py - the control channel between the two halves of a distributed co-sim.

One machine runs the traffic stack (SUMO + TrafficLayer + VirCarlaEnv), the other
runs CARLA. This is how they agree on what to run: newline-delimited JSON over a
single long-lived TCP connection.

Newline-delimited JSON, over one connection held for the whole run, because:

  * The connection IS the liveness signal. It closes -> the CARLA side kills its
    server and goes back to listening. That covers Ctrl+C, a crash and a pulled
    cable without a heartbeat protocol.
  * Cooking a map takes minutes, so the CARLA side has to push progress while the
    traffic side waits. Same socket, no polling endpoint.
  * Standard library only. run_cosim re-execs into a resolved conda env; a new
    dependency is friction exactly where it hurts.
  * It is the idiom already in the tree - TrafficLayer <-> VirCarlaEnv is raw TCP.

DIRECTION: the CARLA host listens, the traffic host dials. The CARLA host already
accepts inbound for CARLA's own RPC ports, so the control port joins that same
firewall rule and the traffic machine opens nothing. Reversed, every traffic box
would need an inbound exception for no gain.

WHAT CROSSES: decisions, never files. The scenario yaml stays on the machine that
owns it - the CARLA side is told which map to serve, not handed a config. Roughly
ten scalars, listed in SERVE/PREPARE below.
"""
import json
import socket
import time

PROTO = 1

# Derived from the CARLA RPC port rather than configured: the listener has to bind
# before it knows which scenario is running, so a yaml field would be a second
# owner of the number (the failure already documented for step_length). Deriving
# keeps CarlaSetup.CarlaServerPort authoritative and gives two CARLA hosts on
# different RPC ports distinct control ports for free. 2400 is clear of CARLA's
# 2000-2002 and TrafficLayer's 420/430/440.
PORT_OFFSET = 400


def peer_port(carla_port):
    return int(carla_port) + PORT_OFFSET


# --------------------------------------------------------------- wire framing

class PeerError(Exception):
    pass


def send(sock, msg):
    sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))


class Reader:
    """Buffered newline-delimited JSON reader over a blocking socket."""

    def __init__(self, sock):
        self.sock = sock
        self.buf = b""

    def read(self, timeout=None):
        """Next message, or None if the peer closed the connection."""
        self.sock.settimeout(timeout)
        while b"\n" not in self.buf:
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                raise PeerError("peer went quiet")
            if not chunk:
                return None                      # closed: the teardown signal
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        try:
            return json.loads(line.decode("utf-8"))
        except ValueError as e:
            raise PeerError(f"peer sent something that is not JSON ({e})")


# ------------------------------------------------------------- traffic side

def connect(host, port, wait=0.0, quiet=False):
    """Dial the CARLA host, retrying for `wait` seconds.

    Retrying is what makes start order not matter: whichever machine comes up
    first waits for the other. A refused connection is not an error until the
    wait is spent."""
    deadline = time.time() + max(wait, 0.0)
    announced = False
    while True:
        try:
            s = socket.create_connection((host, port), timeout=10.0)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            return s
        except OSError:
            if time.time() >= deadline:
                raise PeerError(
                    f"no FIXS peer answering on {host}:{port}. Run "
                    f"'run_cosim --serve' on that machine, or --peer-port on both "
                    f"if {port} is taken.")
            if not announced and not quiet:
                print(f"[peer] waiting for a CARLA peer at {host}:{port} "
                      f"(up to {wait:g}s; Ctrl+C to stop) ...")
                announced = True
            time.sleep(1.0)


def hello(sock, fixs_version, timeout=10.0):
    """Handshake. Returns the peer's WELCOME payload.

    Bounded on purpose: something that is listening but is NOT a FIXS peer would
    otherwise hang us against a stranger's service, which is the worst failure in
    the design - it looks like a slow start rather than a wrong address."""
    r = Reader(sock)
    send(sock, {"t": "HELLO", "proto": PROTO, "fixs_version": fixs_version})
    msg = r.read(timeout)
    if msg is None:
        raise PeerError("peer closed the connection during the handshake")
    if msg.get("t") == "BUSY":
        raise PeerError(f"peer is busy: {msg.get('why', 'serving another run')}")
    if msg.get("t") != "WELCOME":
        raise PeerError(f"something is listening there but did not answer as a "
                        f"FIXS peer (said {msg.get('t')!r})")
    if msg.get("proto") != PROTO:
        raise PeerError(f"peer speaks protocol {msg.get('proto')}, this is {PROTO}; "
                        f"the two machines are on different FIXS builds")
    return r, msg


# ---------------------------------------------------------------- CARLA side

def listen(port, backlog=1):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Without this a restart inside TIME_WAIT reports "address already in use",
    # which reads as "someone else has the port" when nobody does.
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(backlog)
    return srv


def gone(sock):
    """True once the peer has hung up. Non-blocking, so it can be polled from a
    supervision loop next to the process checks.

    MSG_PEEK rather than recv: a BYE sitting in the buffer must still be readable
    afterwards, and either way the answer is the same - the run is over."""
    if sock is None:
        return False
    try:
        import select
        r, _, _ = select.select([sock], [], [], 0)
        if not r:
            return False
        return sock.recv(1, socket.MSG_PEEK) == b""
    except OSError:
        return True


def bye(sock, why=""):
    """Tell the peer we are done, then close. Best effort - the close itself is
    the contract, this only lets the other end say WHY rather than reporting a
    bare disconnection."""
    if sock is None:
        return
    try:
        send(sock, {"t": "BYE", "why": why})
    except OSError:
        pass
    try:
        sock.close()
    except OSError:
        pass


def progress(sock, stage, msg):
    """Push a line to the waiting traffic machine. Best effort - a cook must not
    die because the peer stopped reading."""
    try:
        send(sock, {"t": "PROGRESS", "stage": stage, "msg": msg})
    except OSError:
        pass


def fail(sock, why, retryable=False):
    """Tell the traffic machine this run is not happening, and why. Then close.

    The counterpart to bye(): that one means "it is over", this one means "it
    never started, and this is what stopped it". Both exist so the traffic side is
    never left inferring a cause from a bare disconnection - which is all it used
    to get, in the same sentence, for a map that is not installed, a busy RPC
    port, a failed cook and a pulled cable.

    `retryable` is this side's read on whether another attempt could work. The
    render host returns to listening either way, so it changes no behaviour here;
    it changes the advice the other end can honestly print."""
    if sock is None:
        return
    try:
        send(sock, {"t": "ERROR", "why": str(why), "retryable": bool(retryable)})
    except OSError:
        pass                       # it may be the peer that vanished; nothing to do
    try:
        sock.close()
    except OSError:
        pass
