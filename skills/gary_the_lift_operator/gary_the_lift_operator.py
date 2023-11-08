# Ra-Ya imports
from raya.controllers.cameras_controller import CamerasController
from raya.controllers.cv_controller import CVController
from raya.controllers.arms_controller import ArmsController
from raya.controllers.navigation_controller import NavigationController
from raya.controllers.lidar_controller import LidarController
from raya.controllers.motion_controller import MotionController
from raya.controllers.sound_controller import SoundController
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
import cv2
#import pytesseract
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
        'distance_to_goal' : 0.75
    }


    ###------------------------------ FSM ------------------------------###

    STATES = [
        'APPROACHING_ELEVATOR',
        'DETECTING_BUTTONS_1',
        'MOVING_SIDEWAYS',
        'DETECTING_BUTTONS_2',
        'POSITION_ARM',
        'PRESS_BUTTON',
        'RETURN_ARM',
        'END'
    ]

    INITIAL_STATE = 'APPROACHING_ELEVATOR'

    END_STATES = [
        'END'
    ]
    
    STATES_TIMEOUTS = {'DETECTING_BUTTONS_1' :
                       (NO_TARGET_TIMEOUT, ERROR_BUTTON_NOT_FOUND),

                       'DETECTING_BUTTONS_2' : 
                       (NO_TARGET_TIMEOUT, ERROR_BUTTON_NOT_FOUND),

                       'PRESSING_BUTTON' : 
                       (MOTION_TIMEOUT, ERROR_GOING_BACKWARDS)}

    debug = False
    if debug is True:
        INITIAL_STATE = 'RETURN_ARM'
        #INITIAL_STATE = 'DETECTING_BUTTONS_2'

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
        self.sound = await self.get_controller('sound')
        self.log.info('Sound controller - Enabled')

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
        self.approach_final_linear = 0    # Approach final linear step
        self.approach_angle_error = 0     # Approach final angle error
        self.sideways_distance = 0        # Sideways distance to move
        self.buttons_detected = False     # Buttons detected flag
        self.button_x = None              # x of the button (from baselink)
        self.button_y = None              # y of the button (from baselink)
        self.button_z = None              # z of the button (from baselink)
        self.detection_start_time = None  # Timer for detections
        self.going_backwards_time = None  # Timer for going backwards
        self.position_attempts = 0        # Num of attempts to position the arm
        self.trex_position = None         # Trex pose location
        self.detections_dict = {}         # Dictionary to store detections
        self.image = None                 # Current image from the camera

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



    def reset_detections(self):
        '''Reset the detections'''
        self.buttons_detected = False
        self.detections_dict = {}
        self.button_x, self.button_y, self.button_z = None, None, None
        


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



    async def forward_kinematics(self,
                                 pose,
                                 cartesian_path = True,
                                 planner = 'RRTconnect',
                                 units = ANGLE_UNIT.DEGREES):
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
            cartesian_path = cartesian_path,
            callback_feedback = self.arms_callback_feedback,
            callback_finish = self.arms_callback_finish,
            velocity_scaling = 0.1,
            acceleration_scaling = 0.1,
            wait = True,
            additional_options = {'planner' : planner}
        )



    async def return_arm_home(self):
        await self.arms.set_predefined_pose(
                                arm = self.arm_name,
                                predefined_pose = 'home',
                                callback_feedback = self.arms_callback_feedback,
                                wait = True)

        await self.gripper_command('open')



    async def turn_and_burn(self, distance):
        '''Turn 90 degrees, move forwards, turn back'''
        await self.motion.rotate(angle = 90,
                                 angular_speed = 15,
                                 wait = True)
        
        await self.motion.move_linear(distance = distance,
                                      x_velocity = 0.05,
                                      wait = True)

        await self.motion.rotate(angle = -90,
                                 angular_speed = 15,
                                 wait = True)



    async def static_trex_position(self):
        '''Position arm in trex position'''
        await self.arms.set_joints_position(
            arm=self.arm_name,
            name_joints=self.joint_names,
            angle_joints = TREX_POSITION_ANGLES,
            units = ANGLE_UNIT.RADIANS,
            use_obstacles = True,
            save_trajectory = True,
            name_trajectory = 'trex_position',
            velocity_scaling = 0.4,
            acceleration_scaling =  0.4,
            wait=True)

        self.trex_pose = await self.arms.get_current_pose(self.arm_name)
        self.trex_position = self.trex_pose['position']



    async def dynamic_trex_position(self):
        '''Position arm in trex position'''
        self.trex_pose = {
            'x' : self.button_x + RIGHT_ARM_OFFSET['x'] + GRIPPER_JIG_OFFSET['x'] - 0.1,
            'y' : self.button_y + RIGHT_ARM_OFFSET['y'] + GRIPPER_JIG_OFFSET['y'],
            'z' : self.button_z + RIGHT_ARM_OFFSET['z'] + GRIPPER_JIG_OFFSET['z'],
            'roll' : 0,
            'pitch' : 0,
            'yaw' : 0
        }
        
        self.trex_position = [self.trex_pose['x'],
                              self.trex_pose['y'],
                              self.trex_pose['z']]

        await self.forward_kinematics(self.trex_pose,
                                      #cartesian_path = True,
                                      planner = 'RRTconnect')



    def pixels2meters(self):
        '''Calculate the distance to move sideways from the console'''
        if self.button in self.detections_dict:

            # Convert camera pixels to meters in the current position
            #!!left/right in this position is the X axis for the camera but
            # the Y axis for the base link axes system, hence the names below!!
            x_cam_detection_pix = self.detections_dict[self.button]['object_center_px'][1]
            x_cam_center_dist_pix = abs(MAX_CAMERA_PIXELS_X/2 - x_cam_detection_pix)
            x_cam_edge_dist_pix = abs(MAX_CAMERA_PIXELS_X - x_cam_detection_pix)
            y_base_dist_meters = self.detections_dict[self.button]['center_point'][1]
            pix_meters_ratio = abs(y_base_dist_meters / x_cam_center_dist_pix)

            # Distance so that the button would be on the edge of the screen
            side_linear = float(x_cam_edge_dist_pix * pix_meters_ratio)

            # Semi automatic correction in case of inaccuracies
            if side_linear > 0.4  or side_linear < 0.275:
                side_linear =  0.32 + y_base_dist_meters
            
                return side_linear
    


    def led_confirmation(self):
        '''Confirm that the desired button shows up on the led screen'''

        # Extract the led bounding box
        if 'led' in self.detections_dict:
            x_min = self.detections_dict['led']['x_min']
            x_max = self.detections_dict['led']['x_max']
            y_min = self.detections_dict['led']['y_min']
            y_max = self.detections_dict['led']['y_max']
            led_bbox = self.image[x_min : x_max, y_min : y_max]

            # Binarize image
            led_bbox_gray = cv2.cvtColor(led_bbox, cv2.COLOR_RGB2GRAY)
            _, led_bbox_binary = cv2.threshold(led_bbox_gray, 128, 255,
                                                cv2.THRESH_BINARY)

            # Clean salt and pepper noise using median filter
            led_bbox_no_SP = cv2.medianBlur(led_bbox_binary, 5)

            # Detect the number in the led
            num_in_led = pytesseract.image_to_string(image = led_bbox_no_SP,
                                                     config = '--psm 10')
            
            # Compare the chosen button with the number in the led
            if num_in_led == self.button:
                return True
        
        return False
    
    

    async def sound_control(self, switch):
        '''Turn the beep beep beep sound on or off'''
        if switch is True:
            try:
                await self.sound.play_sound(
                                path = AUDIO_PATH,
                                wait = False,
                                callback_finish = self.sound_callback_finish)
            except Exception as e:
                self.log.warn(f'Couldnt play sound - {e}')

        elif switch is False:
            try:
                await self.sound.cancel_sound()
            except Exception as e:
                self.log.warn(f'Couldnt cancel sound - {e}')

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
        # if 'final_error_angle' in done_info:
        #     self.approach_angle_error = done_info['final_error_angle']



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



    def callback_predictions(self, predictions, image):
        '''Callback used to obtain predictions'''
        self.image = image
        if predictions:
            for pred in predictions:
                object_name = predictions[pred]['object_name']
                self.detections_dict[object_name] = predictions[pred]
        
            if self.button in self.detections_dict:
                self.button_x = self.detections_dict[self.button]['center_point'][0]
                self.button_y = self.detections_dict[self.button]['center_point'][1]
                self.button_z = self.detections_dict[self.button]['center_point'][2]
                self.buttons_detected = True
    
    

    def sound_callback_finish(self, error, error_msg):
        self.log.info(f'msg: {error} | info: {error_msg}')


    ###------------------------------ ACTIONS ------------------------------###

    async def enter_APPROACHING_ELEVATOR(self):
        '''Action used to execute the approach skill'''

        #TODO: REMOVE NAVIGATING AFTER TESTING
        await self.navigation.navigate_to_position(x = 126.0,
                                                   y = 307.0,
                                                   angle = self.execute_args['angle_to_goal'],
                                                   wait = True)

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
                'linear_velocity': 0.06,
                'max_x_error_allowed': 0.03,
                'max_y_error_allowed': 0.02,
                'max_angle_error_allowed' : 3.0,
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


    async def enter_DETECTING_BUTTONS_1(self):
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

        # Start timer
        self.detection_start_time = time.time()


    async def enter_MOVING_SIDEWAYS(self):
        await self.sleep(1.5)
        await self.turn_and_burn(self.sideways_distance)



    async def enter_DETECTING_BUTTONS_2(self):
        # Start timer (model is already enabled)
        self.buttons_detected = False
        await self.sleep(1.5)
        self.detection_start_time = time.time()


    async def enter_POSITION_ARM(self):
        '''Action used to position the arm before pressing the button'''
        
        # Try to position the arm dynamically (according to buttons location)
        try:
            await self.gripper_command('close')
            await self.static_trex_position()
            await self.dynamic_trex_position()

        # Try to position the arm statically (according to const joints values)
        except Exception as e:
            self.log.debug("Couldn't perform dynamic trex positioning. \
                           Performing static trex positioning...")
            try:
                await self.static_trex_position()

            except Exception as e:
                await self.return_arm_home()
                if self.position_attempts > MAX_POSITION_ATTEMPTS:
                    self.log.warn(f'ERROR IN enter_POSITION_ARM - {e}')
                    self.abort(*ERROR_COULDNT_POSITION_ARM)
    


    async def enter_PRESS_BUTTON(self):
        '''Action used to press the chosen button'''
        pose = {
            'x' : self.button_x + RIGHT_ARM_OFFSET['x'] + GRIPPER_JIG_OFFSET['x'],
            'y' : self.button_y + RIGHT_ARM_OFFSET['y'] + GRIPPER_JIG_OFFSET['y'],
            'z' : self.button_z + RIGHT_ARM_OFFSET['z'] + GRIPPER_JIG_OFFSET['z'],
            'roll' : 0,
            'pitch' : 0,
            'yaw' : 0
        }

        try:
            await self.sleep(2.0)
            await self.forward_kinematics(pose)
            await self.sleep(2.0)

        except Exception as e:
            self.log.warn(f'ERROR IN enter_PRESS_BUTTON - {e}. Trying again...')

        
        #TODO: check the led_confirmation feedback mechanism u created
        led_confirmation = True
        if led_confirmation:
            self.button_pressed = True
            self.going_backwards_time = time.time()


    async def enter_RETURN_ARM(self):
        await self.return_arm_home()



    ###---------------------------- TRANSITIONS ----------------------------###
    
    async def transition_from_APPROACHING_ELEVATOR(self):
        if self.approach_successful:
            current_position = await self.navigation.get_position(
                                                pos_unit = POSITION_UNIT.METERS,
                                                ang_unit = ANGLE_UNIT.DEGREES)
            self.approach_angle_error = self.execute_args['angle_to_goal'] - \
                                                            current_position[2] 
            self.set_state('DETECTING_BUTTONS_1')

        else:
            self.set_state('APPROACHING_ELEVATOR')



    async def transition_from_DETECTING_BUTTONS_1(self):
        await self.sleep(1.5)
        if self.buttons_detected:
            self.buttons_detected = False
            await self.motion.rotate(angle = self.approach_angle_error,
                                     angular_speed = 10,
                                     wait = True)
            self.sideways_distance = self.pixels2meters()
            await self.send_feedback(
                {'roation correction' : f'{self.approach_angle_error} degrees',
                 'sideways distance' : f'{self.sideways_distance} meters'}
            )
            self.set_state('MOVING_SIDEWAYS')

        else:
            await self.motion.move_linear(distance = 0.07,
                                            x_velocity = -0.05,
                                            enable_obstacles = False,
                                            wait = False)

        if (time.time() - self.detection_start_time) > NO_TARGET_TIMEOUT:
            self.abort(*ERROR_BUTTON_NOT_FOUND)



    async def transition_from_MOVING_SIDEWAYS(self):
        if not self.motion.is_moving(): 
            self.set_state('DETECTING_BUTTONS_2')



    async def transition_from_DETECTING_BUTTONS_2(self):
        self.reset_detections()
        await self.sleep(1.5)
        if self.buttons_detected:
            self.buttons_detected = False
            self.set_state('POSITION_ARM')
        
        elif (time.time() - self.detection_start_time) > NO_TARGET_TIMEOUT:
            self.abort(*ERROR_BUTTON_NOT_FOUND)

        else:
            try:
                await self.motion.rotate(angle = -7.5,
                                        angular_speed = 5,
                                        wait = False)
            
            except Exception as e:
                self.log.warn(f'ERROR IN transition_from_DETECTING_BUTTONS_2 - {e}')



    async def transition_from_POSITION_ARM(self):
        current_pose = await self.arms.get_current_pose(self.arm_name)
        current_position = np.array(current_pose['position'])
        if all(abs(current_position - self.trex_position) <= ARM_ERROR_THRESHOLD):
            self.set_state('PRESS_BUTTON')

        else:
            self.position_attempts += 1
            if self.position_attempts == MAX_POSITION_ATTEMPTS:
                self.abort(*ERROR_ARM_POSITION_NOT_ACCURATE)
            else:
                self.position_attempts = 0
                self.set_state('POSITION_ARM')



    async def transition_from_PRESS_BUTTON(self):
        await self.sleep(1.0)
        if self.button_pressed and not self.motion.is_moving():
            try:
                await self.motion.move_linear(distance = 0.1,
                                                x_velocity = -0.05,
                                                enable_obstacles = False,
                                                wait = True)
            
                self.set_state('RETURN_ARM')

            except Exception as e:
                if (time.time() - self.going_backwards_time) > MOTION_TIMEOUT:
                    self.log.warn(f'ERROR IN transition_from_PRESS_BUTTON - {e}')
                    self.abort(*ERROR_GOING_BACKWARDS)

        # The button is detected but unreachable, try to rotate a bit
        else:
            self.position_attempts += 1
            if self.position_attempts > MAX_POSITION_ATTEMPTS:
                await self.return_arm_home()
                self.log.warn(f'ERROR IN PRESS_BUTTON - {e}')
                self.abort(*ERROR_PRESSING_BUTTON)
            else:
                if self.button_x >= MAX_CAMERA_PIXELS_X/2:
                    sign = '-'
                if self.button_x < MAX_CAMERA_PIXELS_X/2:
                    sign = '+'
                await self.motion.rotate(angle = float(sign + '7.5'),
                                        angular_speed = 5,
                                        wait = False)

                self.set_state('PRESS_BUTTON')

    async def transition_from_RETURN_ARM(self):
        self.set_state('END')