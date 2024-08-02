
"""
code largely borrowed from robosuite.devices.keyboard
Driver class for Keyboard controller.
"""
import numpy as np
from simulator.core.device import Device
# from utils.transform_utils import rotation_matrix
import carb
import omni

class Keyboard(Device):
    def __init__(self):
        self.command = np.array([.0, .0,])
    
    def start_control(self):
        pass        

    def get_controller_state(self):
        """
        Subscribe to keyboard events
        """
        # subscribe to keyboard events
        app_window = omni.appwindow.get_default_app_window()  # noqa
        key_input = carb.input.acquire_input_interface()  # noqa
        key_input.subscribe_to_keyboard_events(app_window.get_keyboard(), self._sub_keyboard_event)
     
        return self.command

    def _sub_keyboard_event(self, event, *args, **kwargs):
        """subscribe to keyboard events, map to str"""
        # yapf: disable
        if (event.type == carb.input.KeyboardEventType.KEY_PRESS or
                event.type == carb.input.KeyboardEventType.KEY_REPEAT):
            if event.input == carb.input.KeyboardInput.W:
                self.command = np.array([1.0, 0.0])
            if event.input == carb.input.KeyboardInput.S:
                self.command = np.array([-1.0, 0.0])
            if event.input == carb.input.KeyboardInput.A:
                self.command = np.array([0.0, np.pi/2])
            if event.input == carb.input.KeyboardInput.D:
                self.command = np.array([0.0, -np.pi/2])
        if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.command = np.array([0.0, 0.0])


# class Keyboard(Device):
#     """
#     A minimalistic driver class for a Keyboard.
#     Args:
#         pos_sensitivity (float): Magnitude of input position command scaling
#         rot_sensitivity (float): Magnitude of scale input rotation commands scaling
#     """

#     def __init__(self, pos_sensitivity=1.0, rot_sensitivity=1.0):

#         self._display_controls()
#         self._reset_internal_state()

#         self._reset_state = 0
#         self._enabled = False
#         self._pos_step = 0.05

#         self.pos_sensitivity = pos_sensitivity
#         self.rot_sensitivity = rot_sensitivity

#         # make a thread to listen to keyboard and register our callback functions
#         self.listener = Listener(on_press=self.on_press, on_release=self.on_release)

#         # start listening
#         self.listener.start()

#     @staticmethod
#     def _display_controls():
#         """
#         Method to pretty print controls.
#         """

#         def print_command(char, info):
#             char += " " * (30 - len(char))
#             print("{}\t{}".format(char, info))

#         print("")
#         print_command("Keys", "Command")
#         print_command("q", "reset simulation")
#         print_command("spacebar", "toggle gripper (open/close)")
#         print_command("b", "toggle arm/base mode (if applicable)")
#         print_command("up-right-down-left", "move horizontally in x-y plane")
#         print_command(".-;", "move vertically")
#         print_command("o-p", "rotate (yaw)")
#         print_command("y-h", "rotate (pitch)")
#         print_command("e-r", "rotate (roll)")
#         print("")

#     def _reset_internal_state(self):
#         """
#         Resets internal state of controller, except for the reset signal.
#         """
#         self.rotation = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
#         self.raw_drotation = np.zeros(3)  # immediate roll, pitch, yaw delta values from keyboard hits
#         self.last_drotation = np.zeros(3)
#         self.pos = np.zeros(3)  # (x, y, z)
#         self.last_pos = np.zeros(3)
#         self.grasp = False
#         self.base_mode = False

#     def start_control(self):
#         """
#         Method that should be called externally before controller can
#         start receiving commands.
#         """
#         self._reset_internal_state()
#         self._reset_state = 0
#         self._enabled = True

#     def get_controller_state(self):
#         """
#         Grabs the current state of the keyboard.
#         Returns:
#             dict: A dictionary containing dpos, orn, unmodified orn, grasp, and reset
#         """

#         dpos = self.pos - self.last_pos
#         self.last_pos = np.array(self.pos)
#         raw_drotation = (
#             self.raw_drotation - self.last_drotation
#         )  # create local variable to return, then reset internal drotation
#         self.last_drotation = np.array(self.raw_drotation)
#         return dict(
#             dpos=dpos,
#             rotation=self.rotation,
#             raw_drotation=raw_drotation,
#             grasp=int(self.grasp),
#             reset=self._reset_state,
#             base_mode=int(self.base_mode),
#         )
   
