# Nav bottom camera location with respect to baselink (meters)
NAV_BOTTOM_LOCATION = [
    0.148930000000005,
    0,
    0.854009997474796
]

MAX_POSITION_ATTEMPTS = 5

NO_TARGET_TIMEOUT = 10.0
MOTION_TIMEOUT = 10.0

ERROR_BUTTON_NOT_FOUND = (1, 'The chosen button was not found.')
ERROR_PRESSING_BUTTON = (2, 'The location of the button was unreachable.')
ERROR_GOING_BACKWARDS = (3, "Couldn't go backwards to unpress button.")
ERROR_COULDNT_POSITION_ARM = (4, "Couldn't position the arm")
ERROR_ARM_POSITION_NOT_ACCURATE = (5, 'Arm position not accurate')
