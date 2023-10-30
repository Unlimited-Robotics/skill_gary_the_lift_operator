# Ra-Ya imports
from raya.controllers.cameras_controller import CamerasController
from raya.controllers.cv_controller import CVController
from raya.controllers.arms_controller import ArmsController
from raya.controllers.navigation_controller import NavigationController
from raya.controllers.lidar_controller import LidarController
from raya.controllers.motion_controller import MotionController
from raya.controllers.navigation_controller import POSITION_UNIT, ANGLE_UNIT
from gary_arms_msgs.action import CalibrateGripper
from raya.skills import RayaFSMSkill
from skills.approach_to_something import SkillApproachToSomething
from raya.tools.image import show_image, draw_on_image

# Filesystem imports
from skills.gary_the_lift_operator.constants import *
from skills.gary_the_lift_operator.arms_constants import *

# Other imports
import asyncio
import argparse
import time
import numpy as np

class SkillGaryTheLiftOperator(RayaFSMSkill):

    ###------------------------------ SKILL ------------------------------###

    REQUIRED_SETUP_ARGS = [
        'working_camera',
        'map_name',
        'button_to_press'
    ]
    
    DEFAULT_SETUP_ARGS = {
        'fsm_log_transitions': True,
        'identifier': 'blablabla'
    }

    REQUIRED_EXECUTE_ARGS = [
        'angle_to_goal'
    ]

    DEFAULT_EXECUTE_ARGS = {
        'distance_to_goal' : 0.9
    }


    ###------------------------------ FSM ------------------------------###

    STATES = [
        'APPROACHING_ELEVATOR',
        'POSITION_ARM',
        'PRESS_BUTTON',
        'RETURN_ARM',
        'END'
    ]

    INITIAL_STATE = 'APPROACHING_ELEVATOR'

    END_STATES = [
        'END'
    ]
    
    STATES_TIMEOUTS = {}

    debug = False
    if debug is True:
        INITIAL_STATE = 'POSITION_ARM'

    ###--------------------------- SKILL METHODS ---------------------------###

    async def setup(self):
        
        # Setup variables
        self.setup_variables()

        # Get controllers
        self.cameras = await self.get_controller('cameras')
        self.log.info('Cameras controller - Enabled')
        self.cv = await self.get_controller('cv')
        self.log.info('CV controller - Enabled')
        self.navigation = await self.get_controller('navigation')
        self.log.info('Navigation controller - Enabled')
        self.motion = await self.get_controller('motion')
        self.log.info('Motion controller - Enabled')
        self.lidar = await self.get_controller('lidar')
        self.log.info('Lidar controller - Enabled')
        self.arms = await self.get_controller('arms')
        self.log.info('Arms controller - Enabled')

        # Set map
        self.log.info(f"Localizing in map: {self.setup_args['map_name']}...")
        await self.navigation.set_map(
            map_name = self.setup_args['map_name'],
            wait_localization = True,
            wait = True
        )

        # Resgister approach skill
        self.log.info('Registering Helper Skill: ApproachToSomething...')
        self.skill_approach = self.register_skill(SkillApproachToSomething)

        # Setup done log
        self.log.info('Setup Done!')


    async def finish(self):
        pass


    ###------------------------------ HELPERS ------------------------------###

    def setup_variables(self):
        '''Setup initial variables'''

        # General variables
        self.button = str(self.setup_args['button_to_press']) 
        self.approach_successful = False  # Approach success flag
        self.button_x = None              # x of the button (from baselink)
        self.button_y = None              # y of the button (from baselink)
        self.button_z = None              # z of the button (from baselink)
        self.detections_dict = {}

        # Arms variables
        self.arm_name = "right_arm"
        self.joint_names = [
            "arm_right_shoulder_rail_joint",
            "arm_right_shoulder_FR_joint",
            "arm_right_shoulder_RL_joint",
            "arm_right_bicep_twist_joint",
            "arm_right_bicep_FR_joint",
            "arm_right_elbow_twist_joint",
            "arm_right_elbow_FR_joint",
            "arm_right_wrist_joint"
            ]

    def reset_approach_feedbacks(self):
        '''Reset the feedbacks from the approach skill'''
        self.approach_successful = False
    

    async def check_approach_success(self, thresh, max_attempts):
        '''
        INPUTS:
            thresh - the distance under which the robot is close enough (meters)
            max_attempts - the maximum times to try to get closer

        OUTPUTS:
            The function changes the self.approach_successful flag
        '''

        # Get the min distance read from the lidar
        raw_lidar_data = self.lidar.get_raw_data()
        raw_front_data = raw_lidar_data[-10:] + raw_lidar_data[:10]
        min_scan_distance = min(raw_front_data)
        min_actual_distance = min_scan_distance - thresh

        # If you're close enough, return
        if min_actual_distance <= thresh:
            self.approach_successful = True
            return

        # Otherwise, try to move forwards  max_attempts
        else:
            counter = 0
            while counter <= max_attempts:
                await self.motion.move_linear(distance = min_actual_distance,
                                              x_velocity = 0.05)
                raw_lidar_data = self.lidar.get_raw_data()
                min_scan_distance = min(raw_lidar_data)
                min_actual_distance = min_scan_distance - thresh

                if min_actual_distance <= thresh:
                    self.approach_successful = True
                    return

                counter += 1
                
            
        self.approach_successful = False


    def tf_nav_bottom_baselink(self, point):
        '''Transformation between nav_bottom axes syetem to baselink'''
        return NAV_BOTTOM_LOCATION + np.array(point); 


    async def gripper_command(self, command):
        """Opens/closes both grippers"""
        try:
            self.log.info(f'Gripper command \'{command}\'...')
            await self.arms.gripper_cmd(
                **(GRIPPER_COMMANDS[command]),
                wait=True,
            )
        except Exception as e:
            print(e)
        await self.sleep(2)


    async def calibrate_gripper(self, arm):
        """Calibrates gripper on a given arm"""
        # pass
        goal = CalibrateGripper.Goal()
        goal.hand = arm  # side = "right_arm"/"left_arm"
        print(f'calibrating {arm}')
        self.__cli__calibrate_gripper.wait_for_server()
        result = await self.__cli__calibrate_gripper.send_goal_async(goal)  ## await
        print(f'result:{result}')
        print(f'done calibrating {arm}')


    async def forward_kinematics(self, pose, units = ANGLE_UNIT.DEGREES):
        '''
            INPUTS:
                pose: dict with keys of x, y, z, roll, pitch, yaw, and float
                      values

            OUTPUTS:
                The function executes forward kinematics to the desired location
        '''
        
        await self.arms.set_pose(
            arm=self.arm_name,
            x = pose["x"],
            y = pose["y"],
            z = pose["z"],
            roll = pose["roll"],
            pitch = pose["pitch"],
            yaw = pose["yaw"],
            units = units,
            callback_feedback = self.arms_callback_feedback,
            callback_finish = self.arms_callback_finish,
            wait = True,
        )


    async def return_arm_home(self):
        await self.arms.set_predefined_pose(arm = self.arm_name,
                                            predefined_pose = 'home',
                                            wait = True)


    ###----------------------------- CALLBACKS -----------------------------###

    # Create a skill (approach) feedback
    async def skill_callback_feedback(self, feedback):
        self.log.info(f'approach feedback: {feedback}')
        if 'final_linear' in feedback:
            self.approach_final_linear = feedback['final_linear']


    # Create a skill (approach) finish feedback
    async def skill_callback_done(self, done_feedback, done_info):
        self.approach_done_feedback = done_feedback
        self.log.info(f'approach done feedback: {done_feedback}')
        self.log.info(f'approach done info: {done_info}')
        self.detections_dict = done_info['final_detections_dict']

        # Set the final button detections
        if self.button in self.detections_dict:
            
            print('--------------------')
            print(self.detections_dict[self.button])

            # self.button_x = (self.detections_dict[self.button]['x_min'] + \
            #                 (self.detections_dict[self.button]['x_max'] -  
            #                 self.detections_dict[self.button]['x_min'])/2)
            
            # self.button_y = (self.detections_dict[self.button]['y_max'] - \
            #                 self.detections_dict[self.button]['y_min'])/2
            
            # self.button_z = self.detections_dict[self.button]['distance']

            self.button_x = self.detections_dict['center_point'][0]
            self.button_y = self.detections_dict['center_point'][1]
            self.button_z = self.detections_dict['center_point'][2]

            print(f'x y z : {self.button_x}, {self.button_y}, {self.button_z}')

        else:
            self.abort(*ERROR_BUTTON_NOT_FOUND)

    def arms_callback_feedback(self, code, error_feedback, arm, percentage):
        self.log.info(f'ARM:{arm} PERCENTAGE:{percentage:.2f}')


    def arms_callback_finish(self, error, error_msg, fraction):
        self.log.info('')
        if error == 0:
            self.log.info(
                f'FINISH SUCCESSFULLY THE EXECUTION OF THE POSE')
        else:
            self.log.error(
                f'ERROR IN THE EXECUTION NUMBER: {error}:{error_msg}')


    def callback_all_buttons(self, detections, image):
        if detections:
            self.detections_dict['object_name'] = detections['object_name'] 
            self.detections_dict['object_name']['x_max'] = detections['x_max']
            self.detections_dict['object_name']['x_min'] = detections['x_min']
            self.detections_dict['object_name']['y_max'] = detections['y_max']
            self.detections_dict['object_name']['y_min'] = detections['y_min']
            self.detections_dict['object_name']['distance'] = detections['distance']

            print(self.detections_dict)

    ###------------------------------ ACTIONS ------------------------------###

    async def enter_APPROACHING_ELEVATOR(self):
        '''Action used to execute the approach skill'''
        self.reset_approach_feedbacks()
        self.log.info('Executing approach skill...')
        await self.skill_approach.execute_setup(
            setup_args = {
                'working_camera' : self.setup_args['working_camera'],
                'map_name': self.setup_args['map_name'],
                'predictor' : 'elevators_yolov5',
                'identifier': self.setup_args['identifier']
            }
        )
        await self.skill_approach.execute_main(
            execute_args = {
                'angle_to_goal' : self.execute_args['angle_to_goal'],
                'distance_to_goal': self.execute_args['distance_to_goal'],
                'linear_velocity': 0.07,
                'max_x_error_allowed': 0.09,
                'max_y_error_allowed': 0.05,
                'min_correction_distance': 0.1,
                'predictions_to_average' : 2
            },
            wait = False,
            callback_feedback = self.skill_callback_feedback,
            callback_done = self.skill_callback_done
        )

        await self.skill_approach.wait_main()
        await self.skill_approach.execute_finish()
        await self.check_approach_success(
            thresh = self.execute_args['distance_to_goal'] + 0.1,
            max_attempts = 3
            )


    async def enter_POSITION_ARM(self):
        '''Action used to position the arm before pressing the button'''

        pose = {'x' : 0.48661,
                'y' : -0.20621,
                'z' : 1.18,
                'roll' : 0,
                'pitch' : 0,
                'yaw' : 0}
        
        await self.gripper_command('close')
        await self.forward_kinematics(pose = pose)
        await self.return_arm_home()
        await self.gripper_command('open')


    async def enter_PRESS_BUTTON(self):
        '''Action used to press the chosen button'''
        pass

    async def enter_RETURN_ARM(self):
        '''Action used to return the arm home'''
        pass

    ###---------------------------- TRANSITIONS ----------------------------###
    
    async def transition_from_APPROACHING_ELEVATOR(self):
        if self.approach_successful:
            self.set_state('POSITION_ARM')
        else:
            self.set_state('APPROACHING_ELEVATOR')

    
    async def transition_from_POSITION_ARM(self):
        self.set_state('END')


    async def transition_from_PRESS_BUTTON(self):
        pass

    async def transition_from_RETURN_ARM(self):
        pass