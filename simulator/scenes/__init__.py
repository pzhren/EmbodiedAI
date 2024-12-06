from .Interactive_Scene import Interactive_Scene
from simulator.core.register import registry

def make_scene(id_scene,**kwargs):
    scene = registry.get_scene(id_scene)
    assert scene is not None, "Could not find scene with id {}".format(id_scene)
    scene_ins = scene(**kwargs)
    return scene_ins
