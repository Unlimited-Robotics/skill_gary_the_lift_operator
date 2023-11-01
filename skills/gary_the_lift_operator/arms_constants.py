GRIPPER_COMMANDS = {

    'close': {
        'arm': 'both',
        'desired_position': 1.0,
        'desired_pressure': 0.8,
    },

    'open': {
        'arm': 'both',
        'desired_position': 0.0,
        'desired_pressure': 0.8,
    }
}

JOINT_NAMES = [
                "arm_right_shoulder_rail_joint",
                "arm_right_shoulder_FR_joint",
                "arm_right_shoulder_RL_joint",
                "arm_right_bicep_twist_joint",
                "arm_right_bicep_FR_joint",
                "arm_right_elbow_twist_joint",
                "arm_right_elbow_FR_joint",
                "arm_right_wrist_joint"
            ]

RIGHT_ARM_OFFSET_GARY13 = {'x' : -0.04,
                           'y' : -0.02,
                           'z' : 0.007}

ARM_ERROR_THRESHOLD = {'x' : 0.01,
                       'y' : 0.01,
                       'z' : 0.01}