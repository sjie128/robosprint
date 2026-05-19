import time
import lgpio
from rclpy.node import Node

class PickPlaceMechanism:
    def __init__(self, logger):
        self.logger = logger
        try:
            self.handle = lgpio.gpiochip_open(1)
            self.CYLINDER_PIN = 18 # pin num
            lgpio.gpio_claim_output(self.handle, self.CYLINDER_PIN)
        except Exception as e:
            self.logger.error(f"Failed to initialize GPIO: {e}")
            
        self.cube_count = 0

    def junction_action(self, success, cube_detection):
        """Logic for picking cubes at black junctions."""
        if success:
            if cube_detection:
                self.logger.info("real cube. pick up")
                time.sleep(0.5)  # cylinder extend
                lgpio.gpio_write(self.handle, self.CYLINDER_PIN, 0)
                pivot_180_degree()
                lgpio.gpio_write(self.handle, self.CYLINDER_PIN, 1)
                time.sleep(0.3)  # reset
                pivot_180_degree()
                self.cube_count += 1
                return True
            else:
                self.logger.info("fake cube")
                return False
        else:
            self.logger.info("Junction not successfully reached.")
            return False

    def cube_pick_status(self):
        """Check if inventory is full."""
        return self.cube_count >= 3

    def place_cubes(self):
        """Sequence to unload all collected cubes at the target zone."""
        if self.cube_count == 0:
            self.logger.warn("No cubes in inventory to place!")
            return False

        self.logger.info("reach target zone. place cube")

        for i in range(self.cube_count):
            self.logger.info(f"Releasing cube {i}...")
            pivot_180_degree()
            lgpio.gpio_write(self.handle, self.CYLINDER_PIN, 0)
            time.sleep(0.8) # Extend
            lgpio.gpio_write(self.handle, self.CYLINDER_PIN, 1)
            pivot_180_degree()
            time.sleep(0.5) # reset
            self.cube_count -=1
            
        self.cube_count = 0 
        return True