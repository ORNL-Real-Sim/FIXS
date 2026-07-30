"""
FIXS #172: build simple_traffic_light.xosc whose ego FollowTrajectory is the
EXACT path SUMO drove around the loop (sumo_path.txt: x y angle_deg, lane 0, no
lane change). Because the path lies exactly on the real lanes, osc2cm traces the
full loop's lane-paths itself (which build_ego then promotes to a closed Route).

SUMO angle: degrees, 0=North, clockwise -> OpenSCENARIO heading (rad, 0=East, CCW)
= radians(90 - angle).
"""
from __future__ import annotations
import math, pathlib

HERE = pathlib.Path(__file__).resolve().parent
V_MAX = 13.0    # m/s on the straight corridor (~47 km/h)
V_MIN = 5.0     # m/s floor
A_LAT = 1.6     # m/s^2 comfortable lateral accel -> sets curve speed v = sqrt(A_LAT / kappa)


def _angdiff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def main():
    pts = []
    for line in (HERE / "sumo_path.txt").read_text().split("\n"):
        if line.strip():
            x, y, ad, *_ = line.split()
            pts.append((float(x), float(y), math.radians(90.0 - float(ad))))

    # CURVATURE-based speed profile (deterministic, decoupled from SUMO signals):
    # slow through the curved loop, fast on the straight. So FollowTraj no longer
    # barrels through the curve at corridor speed and drift off the road.
    n = len(pts)
    v = [V_MAX] * n
    for i in range(1, n - 1):
        d = math.hypot(pts[i+1][0]-pts[i-1][0], pts[i+1][1]-pts[i-1][1]) / 2.0
        kappa = _angdiff(pts[i+1][2], pts[i-1][2]) / max(d, 0.1)
        vi = math.sqrt(A_LAT / kappa) if kappa > 1e-4 else V_MAX
        v[i] = max(V_MIN, min(V_MAX, vi))
    # smooth (moving average) so the speed changes gradually
    vs = v[:]
    for _ in range(8):
        vs = [vs[0]] + [(vs[i-1]+vs[i]+vs[i+1])/3.0 for i in range(1, n-1)] + [vs[-1]]
    v = vs

    verts, t = [], 0.0
    for i, (x, y, h) in enumerate(pts):
        if i > 0:
            d = math.hypot(x-pts[i-1][0], y-pts[i-1][1])
            t += d / ((v[i] + v[i-1]) / 2.0)
        verts.append((t, x, y, h))
    init_speed_val = v[0]
    vx = "\n".join(
        f'                            <Vertex time="{t:.2f}"><Position>'
        f'<WorldPosition x="{x:.3f}" y="{y:.3f}" z="0" h="{h:.4f}"/></Position></Vertex>'
        for (t, x, y, h) in verts)

    x0, y0, h0 = verts[0][1], verts[0][2], verts[0][3]
    init_speed = init_speed_val   # start at the curvature-profile entry speed
    xosc = f'''<?xml version="1.0" encoding="UTF-8"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="0" date="2026-06-22T00:00:00" description="FIXS #172 SimpleTrafficLight demo (ego path recorded from SUMO, lane 0, no lane change)" author="FIXS"/>
  <ParameterDeclarations><ParameterDeclaration name="owner" parameterType="string" value="Ego"/></ParameterDeclarations>
  <CatalogLocations>
    <VehicleCatalog><Directory path="./Catalogs/Vehicles"/></VehicleCatalog>
    <ControllerCatalog><Directory path="./Catalogs/Controllers"/></ControllerCatalog>
  </CatalogLocations>
  <RoadNetwork><LogicFile filepath="./simple_traffic_light.xodr"/></RoadNetwork>
  <Entities>
    <ScenarioObject name="Ego">
      <CatalogReference catalogName="VehicleCatalog" entryName="car_ego"/>
      <ObjectController><CatalogReference catalogName="ControllerCatalog" entryName="DefaultDriver"/></ObjectController>
    </ScenarioObject>
  </Entities>
  <Storyboard>
    <Init><Actions><Private entityRef="Ego">
      <PrivateAction><LongitudinalAction><SpeedAction>
        <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>
        <SpeedActionTarget><AbsoluteTargetSpeed value="{init_speed:.1f}"/></SpeedActionTarget>
      </SpeedAction></LongitudinalAction></PrivateAction>
      <PrivateAction><TeleportAction><Position>
        <WorldPosition x="{x0:.3f}" y="{y0:.3f}" z="0" h="{h0:.4f}"/>
      </Position></TeleportAction></PrivateAction>
    </Private></Actions></Init>
    <Story name="MyStory"><Act name="MyAct">
      <ManeuverGroup maximumExecutionCount="1" name="MySequence">
        <Actors selectTriggeringEntities="false"><EntityRef entityRef="Ego"/></Actors>
        <Maneuver name="ManeuverEgoLoop"><Event maximumExecutionCount="1" name="EventEgoLoop" priority="overwrite">
          <Action name="ActionEgoFollowLoop"><PrivateAction><RoutingAction>
            <FollowTrajectoryAction>
              <Trajectory name="CorridorLoop" closed="true"><Shape><Polyline>
{vx}
              </Polyline></Shape></Trajectory>
              <TimeReference><Timing domainAbsoluteRelative="absolute" offset="0.0" scale="1.0"/></TimeReference>
              <TrajectoryFollowingMode followingMode="follow"/>
            </FollowTrajectoryAction>
          </RoutingAction></PrivateAction></Action>
          <StartTrigger><ConditionGroup><Condition conditionEdge="none" delay="0" name="Start">
            <ByValueCondition><SimulationTimeCondition rule="greaterThan" value="0"/></ByValueCondition>
          </Condition></ConditionGroup></StartTrigger>
        </Event></Maneuver>
      </ManeuverGroup>
      <StartTrigger><ConditionGroup><Condition name="ActStart" delay="0" conditionEdge="rising">
        <ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan"/></ByValueCondition>
      </Condition></ConditionGroup></StartTrigger>
    </Act></Story>
    <StopTrigger><ConditionGroup><Condition name="End" delay="0" conditionEdge="rising">
      <ByValueCondition><SimulationTimeCondition value="9999" rule="greaterThan"/></ByValueCondition>
    </Condition></ConditionGroup></StopTrigger>
  </Storyboard>
</OpenSCENARIO>
'''
    (HERE / "simple_traffic_light.xosc").write_text(xosc, encoding="utf-8")
    print(f"[gen_ego_traj] wrote .xosc from SUMO path: {len(verts)} vertices, "
          f"loop ~{verts[-1][0]:.0f}s, start ({x0:.0f},{y0:.0f})")


if __name__ == "__main__":
    main()
