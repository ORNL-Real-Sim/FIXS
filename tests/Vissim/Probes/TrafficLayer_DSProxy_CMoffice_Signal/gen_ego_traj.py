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
SPEED = 13.0   # m/s pace along the recorded path


def main():
    pts = []
    for line in (HERE / "sumo_path.txt").read_text().split("\n"):
        if line.strip():
            x, y, ad = map(float, line.split())
            pts.append((x, y, math.radians(90.0 - ad)))

    verts, t = [], 0.0
    for i, (x, y, h) in enumerate(pts):
        if i > 0:
            t += math.hypot(x-pts[i-1][0], y-pts[i-1][1]) / SPEED
        verts.append((t, x, y, h))
    vx = "\n".join(
        f'                            <Vertex time="{t:.2f}"><Position>'
        f'<WorldPosition x="{x:.3f}" y="{y:.3f}" z="0" h="{h:.4f}"/></Position></Vertex>'
        for (t, x, y, h) in verts)

    x0, y0, h0 = verts[0][1], verts[0][2], verts[0][3]
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
        <SpeedActionTarget><AbsoluteTargetSpeed value="{SPEED:.1f}"/></SpeedActionTarget>
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
