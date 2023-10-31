from raya.application_base import RayaApplicationBase
from raya.skills import RayaSkillHandler

from skills.gary_the_lift_operator import SkillGaryTheLiftOperator

# ------------------------------- Application ------------------------------- #
class RayaApplication(RayaApplicationBase):

    async def setup(self):
        self.lift_operator = self.register_skill(SkillGaryTheLiftOperator)
        await self.lift_operator.execute_setup(
            setup_args = {
                'working_camera' : self.camera,
                'map_name' : self.map_name,
                'button_to_press' : self.button_to_press
            }
        )


    async def main(self):
        execute_results = await self.lift_operator.execute_main(
            execute_args = {
                'angle_to_goal' : self.angle_to_goal
            },
            callback_feedback = self.cb_feedback
        )
        self.log.debug(execute_results)


    async def finish(self):
        await self.lift_operator.execute_finish()


# -------------------------------- Helpers -------------------------------- #
    async def cb_feedback(self, feedback):
        self.log.debug(feedback)

    def get_arguments(self):
        self.camera = self.get_argument('-c', '--camera', 
                type = str, 
                required = True,
                help = 'name of camera to use'
            )   
        
        self.angle_to_goal = self.get_argument('-a', '--angle', 
                type = float, 
                required = True,
                help = 'Angle to approach'
            )  
        
        self.map_name = self.get_argument('-m', '--map_name',
                type = str,
                required = True,
                help = 'map name to use'
            )   
        
        self.button_to_press = self.get_argument('-b', '--button_to_press',
                type = str,
                required = True,
                help = 'button to press')