"""#356 probe: the call list, shared by phase A (capture) and phase C (replay),
so the two phases cannot drift apart."""
import traci


def calls():
    return [
        # getters first: the setters below change state
        ("veh_getSpeed",      lambda: traci.vehicle.getSpeed("ego")),
        ("veh_getPosition",   lambda: traci.vehicle.getPosition("ego")),
        ("veh_getRoadID",     lambda: traci.vehicle.getRoadID("ego")),
        ("veh_getRoute",      lambda: traci.vehicle.getRoute("ego")),
        ("veh_getLaneIndex",  lambda: traci.vehicle.getLaneIndex("ego")),
        ("veh_getLeader",     lambda: traci.vehicle.getLeader("ego", 100.0)),
        ("lane_getIDList",    lambda: traci.lane.getIDList()),
        ("sim_getTime",       lambda: traci.simulation.getTime()),
        ("veh_getSpeed_miss", lambda: traci.vehicle.getSpeed("nosuchveh")),
        # setters last
        ("veh_changeLane",    lambda: traci.vehicle.changeLane("ego", 1, 3.0)),
        ("veh_setSpeed",      lambda: traci.vehicle.setSpeed("ego", 5.0)),
        ("veh_setParameter",  lambda: traci.vehicle.setParameter("ego", "probe356", "hello")),
        ("veh_getParameter",  lambda: traci.vehicle.getParameter("ego", "probe356")),
    ]
