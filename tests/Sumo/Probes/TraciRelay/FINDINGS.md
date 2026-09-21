# #356 probe: relaying TraCI over TrafficLayer's existing connection

> **This probe has been acted on.** The relay it measured is implemented in the same
> PR: `CommonLib/TraciRelay.cpp` (the executor), `CommonLib/fixs/traci.py` (the shim),
> and record types 128/129 in `CommonLib/MsgTypes.h`. Tests:
> `tests/Python/unit/test_traci_relay.py` (no simulator) and `tests/Python/TraciRelay/`
> (live co-simulation). This probe is kept because it is the evidence, and because the
> next person to touch this should be able to re-run it rather than re-derive it —
> where a number is quoted in the implementation's comments, it was measured here.

**Verdict: the mechanism works.** Every open question in #356 that could be settled
without building FIXS transport has been settled by running code, not reading it.
A request produced by SUMO's own Python `traci` at `Connection._sendCmd` was executed
by a separate C++ process through `libtraci::Connection::doCommand` on the connection
that process already owned, and the bytes it got back were parsed by the stock
`traci` parsers into the same values a real connection produces — byte-for-byte
identical response payloads, for getters, setters and the error path.

What is **not** probed here: the FIXS hop itself (a `msgType` on the existing record
header, request/response over `SocketHelper`). No FIXS code was written or changed.

## How to reproduce

```
powershell -ExecutionPolicy Bypass -File build_probe.ps1      # needs CommonLib/libsumo
powershell -ExecutionPolicy Bypass -File run_probe.ps1
```

Both take `-LibSumo <path>` if this checkout has no native deps fetched. `run_probe.ps1`
starts two throwaway SUMO runs on `net/probe.sumocfg` (a 2x2-lane 1 km net, three
vehicles, `sigma=0`, `default.speeddev=0` — deterministic, so the two runs see the
same state at the same step).

| phase | what runs | what it proves |
|---|---|---|
| A | `capture.py` — real `traci`, wrapping `Connection._sendCmd` | the four values exist at the seam; records the response bytes traci itself sees |
| B | `probe_relay.exe` — `libtraci`, replays those bytes through `doCommand` | SUMO accepts a payload it never serialized, on a connection the relay did not open |
| C | `replay.py` — stock `traci` domains bound to a fake connection fed with phase B's bytes | the responses parse back into the same values, unmodified parsers |

Result on 2026-09-11, SUMO 1.21.0 server / vendored libtraci 1.22.0 client, MSVC 19.43:
**phase C PASS**, 13/13 calls, all response tails identical.

## What was measured

**1. `_pack` output drops into `doCommand`'s `addData` byte-for-byte.** 13 calls
covering every argument shape that matters: no payload (`getSpeed`), one double
(`getLeader`, `setSpeed`), compound (`changeLane` — `tbd`), compound of strings
(`setParameter` — `tss`), string (`getParameter`), and an empty object id
(`lane.getIDList`, `simulation.getTime`). Example, `changeLane("ego", 1, 3.0)`:

```
cmdID=0xc4 varID=0x13 objID="ego"
payload=0f0000000208010b4008000000000000     <- python _pack, handed to SUMO unread by C++
```

The framing rules agree because both sides implement the same ones:
`connection.py:_sendCmd` and `libtraci/Connection.cpp:createCommand` compute the same
length, use the same one-byte / zero-plus-four-byte form, and append the payload
verbatim after `(cmdID, varID, objID)`.

**2. A relayed setter really lands in SUMO.** Not just "status OK": phase B relayed
`setParameter("ego","probe356","hello")` and then `getParameter` came back `'hello'`,
and the ego state one step after the relayed `changeLane`/`setSpeed` is identical in
phase B and phase A (`12.29,0` both).

**3. The response lines up with traci's parsers with no re-framing**, provided the
relay passes `expectedType = -1`. `doCommand` then consumes only the status message
and returns `myInput` positioned at the start of the value response — exactly where
`Domain._getCmd` expects to start reading (`readLength`, response id, varID, objID).
So the shim's `_sendCmd` can return `Storage(bytes_from_cpp)` and every domain parser
downstream is untouched. Phase C ran the real `traci.vehicle.getLeader`,
`traci.lane.getIDList` etc. against exactly those bytes. (The issue's sketch passes
`0`; that would make `doCommand` eat part of the response header and the Python side
would have to re-synthesize it. Use `-1`.)

**4. The error path relays as an exception, with SUMO's own message.**
`getSpeed("nosuchveh")` → `doCommand` throws `libsumo::TraCIException`
(`"Vehicle 'nosuchveh' is not known."`); phase C re-raised it as `traci.TraCIException`
and the caller cannot tell the difference.

**5. Relaying does not disturb the subscriptions TrafficLayer's feed is built on.**
The probe subscribes (`VAR_SPEED`, `VAR_LANE_INDEX`) before relaying anything; after
13 relayed commands and a step, the subscription results still match the typed getters.

**6. The relay can run off the tick thread if it holds `Connection::getMutex()`.**
libtraci's own domains take that mutex around every `doCommand`, and
`Connection::simulationStep` takes it too — so the discipline is "caller locks", and a
relay that follows it is no different from a domain call. Measured: 300 relayed getters
on a second thread against 300 `Simulation::step()` on the main thread, clean, no
corruption. The relay therefore does **not** have to be pinned to a point in the tick.

**7. Cost is the SUMO round trip and nothing else.** 2000 relayed `getSpeed` vs 2000
`libtraci::Vehicle::getSpeed`: **31–36 µs vs 29–40 µs** per call, i.e. the same within
noise. `doCommand` *is* the path libtraci itself uses, so the relay adds no C++-side
cost; whatever a TraCI-shaped script costs today it will cost through the relay, plus
the FIXS hop (unmeasured). The per-tick budget argument in #356 stands — subscriptions
should be the obvious path — but the relay is not what makes getters expensive.

**8. The refuse list is load-bearing, not defensive.** `CMD_SIMSTEP` fits the
four-value contract as well as `getSpeed` does: the probe relayed
`doCommand(0x02, -1, "", <raw double>)` and the clock moved `35.10 -> 35.20` behind the
owning process's back. Nothing in the mechanism prevents this; only a refusal does.

**9. Two shapes reach `_sendCmd` that the four-value contract cannot express**, so the
shim must branch on them rather than forward them:

```
simulationStep    cmdID=0x02  varID=None                  payload=0000000000000000
subscribe         cmdID=0xd4  varID=(begin, end) tuple    payload=024042
subscribeContext  cmdID=0x84  varID=(begin, end) tuple    payload=a440490000000000000140
```

`varID=None` (no varID and no objID on the wire) *is* expressible as `varID=-1` in
`doCommand` — that is how `CMD_SIMSTEP` got through in finding 8 — but a subscription's
`varID` is a *pair of doubles*, and `doCommand` has no room for it.
`Connection::subscribe` is exported separately and takes
`(domID, objID, begin, end, domain, range, vars, params)`; relaying subscriptions means
a second message shape, not the same one.

## Packaging: the real obstacle

The mechanism is fine; **shipping it is the part that needs a decision.**

- `foreign/tcpip/storage.h` **and `storage.cpp`** must be compiled into the relaying
  process. `tcpip::Storage` is essentially *not exported* from `libtracicpp.dll` — of
  the exports, exactly one is a `Storage` member (`size()`). The probe compiles the
  1.22.0 `storage.cpp` itself (`build_probe.ps1` fetches it, pinned to the tag) and
  passes the resulting object across the DLL boundary; that works because both sides
  are MSVC release `/MD` with the same STL layout, and because `Storage`'s methods are
  virtual, so the returned DLL-owned object dispatches into the DLL's own code.
  Adding both files to the native-deps zip (`pack_native_deps.ps1`) is the minimum.
- **`libtraci/Connection.h` cannot simply be vendored.** It includes
  `libsumo/Subscription.h`, which includes `utils/common/SUMOVehicleClass.h` and
  `utils/common/SUMOTime.h` — neither is in the shipped set, and pulling them in drags
  a large part of SUMO's `utils/common` behind them. The probe therefore declares
  `libtraci::Connection` locally with just the three members it calls, which links
  against `libtracicpp.lib` because MSVC mangling of a member function depends only on
  the class name, namespace and signature. It works — it is how this probe ran — but it
  binds FIXS to MSVC name mangling and to these signatures. The honest options are
  (a) vendor a trimmed `Connection.h`, (b) keep a local declaration and pin it with a
  build-time check, or (c) ask upstream for a stable C-shaped entry point. **Unresolved.**
- `getActive()` is exported out-of-line by the vendored **1.22** DLL but **not** by the
  1.21.0 install (it is `inline` in both versions' headers; 1.22's build happened to
  emit it). Code that relies on that export is version-fragile. `myActive` is exported
  as data in both, so a local declaration of `getActive()` links today — for 1.22.

## Still open

- The FIXS hop: msgType framing, request/response over `SocketHelper`, and what the
  relay costs once it is inside the lockstep. Nothing here measures that.
- Subscriptions through the relay (see finding 9) — a separate message shape.
- The libsumo build (`ENABLE_LIBSUMO`): there is no `Connection` and no generic
  executor, so there is no relay. Refuse with a clear message, or fall back to a feed
  facade — still a decision, not a finding.
- Everything measured here is one SUMO version pair (1.21.0 server, 1.22.0 client) on
  one machine.
