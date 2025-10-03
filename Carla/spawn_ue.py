import unreal
import csv

actors = unreal.EditorLevelLibrary.get_all_level_actors()


### Spawn Traffic Lights and TL Group according to intersection

bp_path = '/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLight.BP_TrafficLight'
blueprint_path = unreal.EditorAssetLibrary.load_blueprint_class(bp_path)

tl_bp_path = '/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLightGroup.BP_TrafficLightGroup'
tl_blueprint_path = unreal.EditorAssetLibrary.load_blueprint_class(tl_bp_path)

jn_spawned = {}
id = 1

with open('/home/arms/carlatlgen/carla_tl_locs.csv', newline='') as csvfile:
    csv_reader = csv.reader(csvfile)

    for row in csv_reader:

        if row[6] not in jn_spawned:
            jn_spawned[row[6]] = True
            location = unreal.Vector()
            location.x = float(row[6]) * 100
            location.y = float(row[7]) * 100
            location.z = float(row[8]) * 100
            rotation = unreal.Rotator(0,0,0)
            actor = unreal.EditorLevelLibrary.spawn_actor_from_class(tl_blueprint_path, location, rotation)
            actor.set_actor_label(f'BP_TrafficLightGroup{row[-1]}')

        location = unreal.Vector()
        location.x = float(row[0]) * 100
        location.y = float(row[1]) * 100
        location.z = float(row[2]) * 100
        rotation = unreal.Rotator(float(row[3]), 
                                  float(row[4]), 
                                  float(row[5]))


        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(blueprint_path, location, rotation)
        actor.set_actor_label(f'{row[-2]}')
