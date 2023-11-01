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
        'button_to_press',
    ]
    
    DEFAULT_SETUP_ARGS = {
        'fsm_log_transitions': True,
        'identifier': 'blablabla',
        'arm_name' : 'right_arm'
    }

    REQUIRED_EXECUTE_ARGS = [
        'angle_to_goal'
    ]

    DEFAULT_EXECUTE_ARGS = {
        'distance_to_goal' : 0.7
    }


    ###------------------------------ FSM ------------------------------###

    STATES = [
        'APPROACHING_ELEVATOR',
        'MOVING_SIDEWAYS',
        'DETECTING_BUTTONS',
        'POSITION_ARM',
        'PRESS_BUTTON',
        'RETURN_ARM',
        'END'
    ]

    INITIAL_STATE = 'APPROACHING_ELEVATOR'

    END_STATES = [
        'END'
    ]
    
    STATES_TIMEOUTS = {'DETECTING_BUTTONS' :
                       (NO_TARGET_TIMEOUT, ERROR_BUTTON_NOT_FOUND),

                       'PRESSING_BUTTON' : 
                       (MOTION_TIMEOUT, ERROR_GOING_BACKWARDS)}

    debug = False
    if debug is True:
        INITIAL_STATE = 'RETURN_ARM'

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
        self.buttons_detected = False     # Buttons detected flag
        self.button_x = None              # x of the button (from baselink)
        self.button_y = None              # y of the button (from baselink)
        self.button_z = None              # z of the button (from baselink)
        self.detection_start_time = None  # Timer for detections
        self.going_backwards_time = None  # Timer for going backwards
        self.position_attempts = 0        # Num of attempts to position the arm
        self.detections_dict = {}         # Dictionary to store detections

        # Arms variables
        self.arm_name = self.setup_args['arm_name']
        self.joint_names = JOINT_NAMES



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
        result = await self.__cli__calibrate_gripper.send_goal_async(goal)
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
            cartesian_path = True,
            callback_feedback = self.arms_callback_feedback,
            callback_finish = self.arms_callback_finish,
            wait = True,
            additional_options = {'planner' : 'RRTstar'}
        )



    async def return_arm_home(self):
        await self.arms.set_predefined_pose(
                                arm = self.arm_name,
                                predefined_pose = 'home',
                                callback_feedback = self.arms_callback_feedback,
                                wait = True)

        await self.gripper_command('open')



    async def turn_and_burn(self):
        '''Turn 90 degrees, move forwards, turn back'''
        await self.motion.rotate(angle = 90, angular_speed = 15, wait = True)
        await self.motion.move_linear(distance = 0.225, x_velocity = 0.05, wait = True)
        await self.motion.rotate(angle = -90, angular_speed = 15, wait = True)



    async def trex_position(self):
        '''Position arm in trex position'''
        angles = [0.0, 0.0, 0.0, 0.0, 2.1, 0.0, -0.5, 1.57]
        await self.arms.set_joints_position(
            arm=self.arm_name,
            name_joints=self.joint_names,
            angle_joints = angles,
            units = ANGLE_UNIT.RADIANS,
            use_obstacles = True,
            save_trajectory = True,
            name_trajectory = 'trex_position',
            velocity_scaling = 0.5,
            acceleration_scaling =  0.8,
            wait=True)



    async def test_trex_position(self):
        '''Position arm in trex position'''
        self.trex_pose = {'x' : 0.1,
                'y' : self.button_y + RIGHT_ARM_OFFSET_GARY13['y'],
                'z' : self.button_z + RIGHT_ARM_OFFSET_GARY13['z'],
                'roll' : 0,
                'pitch' : 0,
                'yaw' : 0}

        await self.forward_kinematics(self.trex_pose)



    ###----------------------------- CALLBACKS -----------------------------###

    async def skill_callback_feedback(self, feedback):
        '''ApproachToSomething skill feedback callback'''
        self.log.info(f'approach feedback: {feedback}')
        if 'final_linear' in feedback:
            self.approach_final_linear = feedback['final_linear']



    async def skill_callback_done(self, done_feedback, done_info):
        '''ApproachToSomething skill finish callback'''
        self.approach_done_feedback = done_feedback
        self.log.info(f'approach done feedback: {done_feedback}')
        self.log.info(f'approach done info: {done_info}')
        


    def arms_callback_feedback(self, code, error_feedback, arm, percentage):
        self.log.info(f'ARM: {arm} TRAJECTORY: {percentage:.2f}% DONE')



    def arms_callback_finish(self, error, error_msg, fraction):
        self.log.info('')
        if error == 0:
            self.log.info(
                f'FINISH SUCCESSFULLY THE EXECUTION OF THE POSE')
        else:
            self.log.error(
                f'ERROR IN THE EXECUTION NUMBER: {error}:{error_msg}')



    def callback_predictions(self, predictions, timestamp):
        '''Callback used to obtain predictions'''
        if predictions:
            for pred in predictions:
                object_name = predictions[pred]['object_name']
                self.detections_dict[object_name] = predictions[pred]
        
            if self.button in self.detections_dict:
                self.button_x = self.detections_dict[self.button]['center_point'][0]
                self.button_y = self.detections_dict[self.button]['center_point'][1]
                self.button_z = self.detections_dict[self.button]['center_point'][2]

                self.buttons_detected = True
                


    ###------------------------------ ACTIONS ------------------------------###

    async def enter_APPROACHING_ELEVATOR(self):
        '''Action used to execute the approach skill'''
        self.reset_approach_feedbacks()
        self.log.info('Executing ApproachToSomething skill...')
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
                'max_x_error_allowed': 0.07,
                'max_y_error_allowed': 0.03,
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



    async def enter_MOVING_SIDEWAYS(self):
        await self.sleep(2.0)
        await self.turn_and_burn()



    async def enter_DETECTING_BUTTONS(self):

         # Start timer
        self.detection_start_time = time.time()

        # Enable model
        self.log.info('Enabling elevators_yolov5 model...')
        self.predictor_handler = await self.cv.enable_model(
                name = 'elevators_yolov5', 
                source = self.setup_args['working_camera'],
                model_params = {'depth':True}
            )
        
        # Set handler
        self.predictor_handler.set_detections_callback(
                callback=self.callback_predictions,
                as_dict=True,
                call_without_detections=True
            )
    


    async def enter_POSITION_ARM(self):
        '''Action used to position the arm before pressing the button'''
        #await self.trex_position()
        try:   
            await self.test_trex_position()
            await self.gripper_command('close')

        except Exception as e:
            await self.return_arm_home()
            self.abort(*ERROR_COULDNT_POSITION_ARM)
        


    async def enter_PRESS_BUTTON(self):
        '''Action used to press the chosen button'''
        pose = {'x' : self.button_x + RIGHT_ARM_OFFSET_GARY13['x'],
                'y' : self.button_y + RIGHT_ARM_OFFSET_GARY13['y'],
                'z' : self.button_z + RIGHT_ARM_OFFSET_GARY13['z'],
                'roll' : 0,
                'pitch' : 0,
                'yaw' : 0}

        try:
            await self.forward_kinematics(pose)

        except Exception as e:
            await self.return_arm_home()
            self.log.warn(f'ERROR IN PRESS_BUTTON - {e}')
            self.abort(*ERROR_PRESSING_BUTTON)
        
        #TODO: add a feedback mechanism
        feedback_mechanism = True
        if feedback_mechanism:
            self.button_pressed = True
            self.going_backwards_time = time.time()


    async def enter_RETURN_ARM(self):
        await self.return_arm_home()



    ###---------------------------- TRANSITIONS ----------------------------###
    
    async def transition_from_APPROACHING_ELEVATOR(self):
        if self.approach_successful:
            self.set_state('MOVING_SIDEWAYS')
        else:
            self.set_state('APPROACHING_ELEVATOR')


    
    async def transition_from_MOVING_SIDEWAYS(self):
        if not self.motion.is_moving(): 
            self.set_state('DETECTING_BUTTONS')



    async def transition_from_DETECTING_BUTTONS(self):
        if self.buttons_detected:
            self.log.info('Disabling model elevators_yolov5...')
            await self.cv.disable_model(model_obj = self.predictor_handler)
            self.set_state('POSITION_ARM')
        
        elif (time.time() - self.detection_start_time) > NO_TARGET_TIMEOUT:
            self.abort(*ERROR_BUTTON_NOT_FOUND)



    async def transition_from_POSITION_ARM(self):
        current_pose = self.arms.get_current_pose()
        self.log.debug(current_pose)
        if current_pose - self.trex_pose <= ARM_ERROR_THRESHOLD:
            self.set_state('PRESS_BUTTON')

        else:
            self.position_attempts += 1
            if self.position_attempts == MAX_POSITION_ATTEMPTS:
                self.abort(*ERROR_ARM_POSITION_NOT_ACCURATE)
            self.set_state('POSITION_ARM')



    async def transition_from_PRESS_BUTTON(self):
        if self.button_pressed and not self.motion.is_moving():
            try:
                await self.motion.move_linear(distance = 0.1,
                                                x_velocity = -0.05,
                                                enable_obstacles = True,
                                                wait = True)
            
                self.set_state('RETURN_ARM')

            except Exception as e:
                if (time.time() - self.going_backwards_time) > MOTION_TIMEOUT:
                    self.log.debug(f'Got error - {e}')
                    self.abort(*ERROR_GOING_BACKWARDS)



    async def transition_from_RETURN_ARM(self):
        self.set_state('END')