### Official asset path

```python
from omni.isaac.nucleus import get_assets_root_path

assets_root_path = get_assets_root_path()
```

### For example: 

robot path can be found in app UI.

```python
if assets_root_path is None:

    carb.log_error("Could not find Isaac Sim assets folder")

jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
```

### Add robot 

```python
my_jetbot = WheeledRobot(

        prim_path="/World/Jetbot",

        name="my_jetbot",

        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],

        create_robot=True,

        usd_path=jetbot_asset_path,

        position=np.array([1.6, 1.6, 0.5]),

    )

my_jetbot = my_world.scene.add(my_jetbot)

```

### Add controller

```python
my_controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
```

### Note!!! The above operations will only take effect after reset.

```python
from omni.isaac.core import World

my_world = World(stage_units_in_meters=1.0)
my_world.reset()
```


### Here is an example showing how to use the controller to control the movement of robot joints. 

```python
hile simulation_app.is_running():

    my_world.step(render=True)

    if my_world.is_stopped() and not reset_needed:

        reset_needed = True

    if my_world.is_playing():

        if reset_needed:

            my_world.reset()

            my_controller.reset()

            reset_needed = False

        if i >= 0 and i < 300:

            position, orientation = my_jetbot.get_local_pose()

            print(position)

            my_jetbot.apply_wheel_actions(my_controller.forward(command=[0.3, np.pi/2]))

        if i >= 300 and i < 500:

            position, orientation = my_jetbot.get_local_pose()

            print(position)

            my_jetbot.apply_wheel_actions(my_controller.forward(command=[0.2, -np.pi/2]))

        if i>= 500:

            position, orientation = my_jetbot.get_local_pose()

            my_jetbot.apply_wheel_actions(my_controller.forward(command=[0.2, np.pi/2]))

            print(position)
```


More details can be found in EmbodiedAI/tests/isaac-sim_test.py