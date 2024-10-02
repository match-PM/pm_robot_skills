import rclpy
from rclpy.node import Node

from pm_skills_interfaces.srv import VacuumGripper, DispenseAdhesive, ConfocalLaser, ExecuteVision
import time
import random

class PmSkills(Node):
    def __init__(self) -> None:
        super().__init__('pm_skills')

        self.vakuum_service = self.create_service(VacuumGripper, "pm_skills/vacuum_gripper", self.vacuum_gripper_callback)
        self.dispenser_service = self.create_service(DispenseAdhesive, "pm_skills/dispense_adhesive", self.dispenser_callback)
        self.confocal_laser_service = self.create_service(ConfocalLaser, "pm_skills/confocal_laser", self.confocal_laser_callback)
        self.vision_service = self.create_service(ExecuteVision, "pm_skills/execute_vision", self.vision_callback)
        self.logger = self.get_logger()

    def vacuum_gripper_callback(self, request, response):
        """TO DO: Add docstring"""

        response.success = True

        if request.vacuum:
            self.logger.info("Vacuum gripper activated!")
        else:
            self.logger.info("Vacuum gripper deactivated!")
        
        return response

    def dispenser_callback(self, request, response):
        """TO DO: Add docstring"""

        self.logger.info(f"Starting {request.dispenser}")

        # Simulating dispension time by adding delay
        wait_duration = float(request.dispense_time)
        if wait_duration > 0.0:
            time.sleep(wait_duration)
            response.success = True
        else:
            self.logger.warn("Invaild dispense Duration!")
            response.success = False
            
        self.logger.info(f"Done {request.dispenser}")
        return response

    def confocal_laser_callback(self, request, response):
        """TO DO: Add docstring"""

        self.logger.info(f"Starting laser: {request.laser}")

        # Generating random number for meassurement    
        result = ((random.randrange(0, 6)) / 10.0) - 0.3

        response.result = str(f"{result} mm.")
        response.success = True

        return response
    
    def vision_callback(self, request, response):
        self.logger.info(f'Loading {request.process_filename} as process file.')
        self.logger.info(f'Starting process {request.process_uid}')
        self.logger.info(f"Loading {request.camera_config_filename} as camera configuration file.")

        if request.image_display_time > 0.0:
            self.logger.info('Image aquisiton is running')
            time.sleep(request.image_display_time)
            response.success = True
        else: 
            self.logger.warn('Invalid image display time!')
            response.success = False

        if request.run_cross_validation:
            self.logger.info("Running cross validtation.")

        response.results_dict = "Result dict"
        self.logger.info(f'Saving results ...')
        response.results_path = "Results path."
        response.points = [random.random()*100 for _ in range(4)]
        response.process_uid = str(random.randint(1000,50000))

        return response
            


def main(args=None):
    rclpy.init(args=args)

    pm_skills = PmSkills()

    rclpy.spin(pm_skills)

    rclpy.shutdown()



if __name__ == '__main__':
    main()
