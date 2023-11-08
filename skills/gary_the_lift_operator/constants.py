# Nav bottom camera location with respect to baselink (meters)
NAV_BOTTOM_LOCATION = [
    0.148930000000005,
    0,
    0.854009997474796
]

# Path for the beep audio when pressing the button
AUDIO_PATH = f'dat:audio/beep_sound.mp3'

# The X resolution of the camera (nav bottom)
MAX_CAMERA_PIXELS_X = 865

# Distance from base link to the arms (meters)
BASE_TO_ARM_DISTANCE = 0.23

MAX_POSITION_ATTEMPTS = 5

NO_TARGET_TIMEOUT = 10.0
MOTION_TIMEOUT = 10.0

ERROR_BUTTON_NOT_FOUND = (1, 'The chosen button was not found.')
ERROR_PRESSING_BUTTON = (2, 'The location of the button was unreachable.')
ERROR_GOING_BACKWARDS = (3, "Couldn't go backwards to unpress button.")
ERROR_COULDNT_POSITION_ARM = (4, "Couldn't position the arm")
ERROR_ARM_POSITION_NOT_ACCURATE = (5, 'Arm position not accurate')
