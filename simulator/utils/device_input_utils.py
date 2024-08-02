import numpy as np
import utils.transform_utils as T

def input2action(device, robot="jetbot", active_arm="right", env_configuration=None, mirror_actions=False):
    state = device.get_controller_state()

    dpos, rotation, raw_drotation, grasp, reset = (
        state["dpos"],
        state["rotation"],
        state["raw_drotation"],
        state["grasp"],
        state["reset"],
    )

    if mirror_actions:
        dpos[0], dpos[1] = dpos[1], dpos[0]
        raw_drotation[0], raw_drotation[1] = raw_drotation[1], raw_drotation[0]

        dpos[1] *= -1
        raw_drotation[0] *= -1

    if reset:
        return None, None

    drotation *= 10
    dpos *= 5
    # relative rotation of desired from current eef orientation
    # map to quat
    drotation = T.mat2quat(T.euler2mat(drotation))
     
    # test jetbot keyboard
    if robot=="jetbot":
        base_mode = bool(state["base_mode"])
        if base_mode is True:
            move_action = np.clip(np.array([dpos[0],dpos[1]]),-1,1)
            

