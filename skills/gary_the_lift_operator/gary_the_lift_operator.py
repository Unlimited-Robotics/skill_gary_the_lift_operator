# Ra-Ya imports
from raya.controllers.cameras_controller import CamerasController
from raya.controllers.cv_controller import CVController
from raya.controllers.arms_controller import ArmsController
from raya.controllers.navigation_controller import NavigationController
from raya.controllers.lidar_controller import LidarController
from raya.controllers.motion_controller import MotionController

from raya.skills import RayaFSMSkill
from skills.approach_to_something import SkillApproachToSomething

# Other imports
import asyncio
import argparse
import time

class SkillGaryTheLiftOperator(RayaFSMSkill):

    ###------------------------------ SKILL ------------------------------###

    REQUIRED_SETUP_ARGS = [
        'working_camera',
        'map_name',
    ]
    
    DEFAULT_SETUP_ARGS = {
        'fsm_log_transitions': True,
        'identifier': 'blablabla'
    }

    REQUIRED_EXECUTE_ARGS = [
        'angle_to_goal'
    ]

    DEFAULT_EXECUTE_ARGS = {
        'distance_to_goal' : 0.8
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

        # Approach skill setup
        self.log.info('Registering Helper Skill: ApproachToSomething...')
        self.skill_approach = self.register_skill(SkillApproachToSomething)
        await self.skill_approach.execute_setup(
            setup_args = {
                'working_camera' : self.setup_args['working_camera'],
                'map_name': self.setup_args['map_name'],
                'predictor' : 'elevators_yolov5',
                'identifier': self.setup_args['identifier']
            }
        )

        # Setup done log
        self.log.info('Setup Done!')


    async def finish(self):
        pass


    ###------------------------------ HELPERS ------------------------------###

    def setup_variables(self):
        '''Setup initial variables'''
        self.approach_successful = False

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

    def mimic_predefined_pose(self, arm, name):
        '''Execute a tranjectory for a single arm'''

        # Check arm position is valid
        pass

    ###----------------------------- CALLBACKS -----------------------------###

    # Create a skill (approach) feedback
    async def skill_cb_feedback(self, feedback):
        self.log.info(f'approach feedback: {feedback}')
        if 'final_linear' in feedback:
            self.approach_final_linear = feedback['final_linear']

    # Create a skill (approach) finish feedback
    async def skill_cb_done(self, done_feedback, done_info):
        self.approach_done_feedback = done_feedback
        self.log.info(f'approach done feedback: {done_feedback}')
        self.log.info(f'approach done info: {done_info}')


    ###------------------------------ ACTIONS ------------------------------###

    async def enter_APPROACHING_ELEVATOR(self):
        '''Action used to execute the approach skill'''
        self.reset_approach_feedbacks()
        await self.skill_approach.execute_main(
            execute_args = {
                'angle_to_goal' : self.execute_args['angle_to_goal'],
                'distance_to_goal': self.execute_args['distance_to_goal'],
                'linear_velocity': 0.07,
                'max_x_error_allowed': 0.09,
                'max_y_error_allowed': 0.05,
                'predictions_to_average' : 2
            },
            wait = False,
            callback_feedback = self.skill_cb_feedback,
            callback_done = self.skill_cb_done
        )

        await self.skill_approach.wait_main()
        await self.skill_approach.execute_finish()
        await self.check_approach_success(
            thresh = self.execute_args['distance_to_goal'] + 0.1,
            max_attempts = 3
            )


    async def enter_POSITION_ARM(self):
        '''Action used to position the arm to press the button'''
        await self.mimic_predefined_pose(arm = 'right_arm', name = 'pose_1')

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
        pass

    async def transition_from_PRESS_BUTTON(self):
        pass

    async def transition_from_RETURN_ARM(self):
        pass