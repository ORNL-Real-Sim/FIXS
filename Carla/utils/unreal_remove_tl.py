import unreal

actors = unreal.EditorLevelLibrary.get_all_level_actors()

for actor in actors:
    label = actor.get_actor_label()
    if label.startswith("TrafficLightGroup") or label.startswith("TrafficLight"):
        unreal.EditorLevelLibrary.destroy_actor(actor)

unreal.EditorLoadingAndSavingUtils.save_dirty_packages(True, True)
