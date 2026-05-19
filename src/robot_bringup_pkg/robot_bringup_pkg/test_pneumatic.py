import time
import lgpio
import rclpy
from rclpy.node import Node

# --- MOCK FUNCTION FOR TESTING ---
def pivot_180_degree():
    print("[MOCK] Chassis/Servo pivoting 180 degrees...")
    time.sleep(0.5)

class PickPlaceMechanism:
    def __init__(self, logger):
        self.logger = logger
        try:
            # NOTE: On Raspberry Pi 4/5, gpiochip_open(0) is typically used for standard GPIOs.
            # Change back to 1 if you are using an external hardware overlay expander.
            self.handle = lgpio.gpiochip_open(0)
            self.CYLINDER_PIN = 18 
            lgpio.gpio_claim_output(self.handle, self.CYLINDER_PIN)
            
            # Set initial safe state (1 usually turns off a relay module, 0 turns it on)
            lgpio.gpio_write(self.handle, self.CYLINDER_PIN, 1)
        except Exception as e:
            self.logger.error(f"Failed to initialize GPIO: {e}")
            
        self.cube_count = 0

    def junction_action(self, success, cube_detection):
        """Logic for picking cubes at black junctions."""
        if success:
            if cube_detection:
                self.logger.info("REAL CUBE DETECTED! Starting pickup loop...")
                
                # Turn relay ON (fires pneumatic)
                lgpio.gpio_write(self.handle, self.CYLINDER_PIN, 0)
                time.sleep(0.5)  
                
                pivot_180_degree()
                
                # Turn relay OFF (retracts pneumatic)
                lgpio.gpio_write(self.handle, self.CYLINDER_PIN, 1)
                time.sleep(0.3)  
                
                pivot_180_degree()
                self.cube_count += 1
                self.logger.info(f"Inventory status: {self.cube_count} cube(s) held.")
                return True
            else:
                self.logger.info("FAKE CUBE DETECTED! Skipping hardware activation.")
                return False
        else:
            self.logger.info(" Junction not successfully reached.")
            return False

    def cube_pick_status(self):
        return self.cube_count >= 3

    def place_cubes(self):
        if self.cube_count == 0:
            self.logger.warn("No cubes in inventory to place!")
            return False

        self.logger.info("Target zone reached! Launching unloading sequence...")

        # Loop through whatever inventory count we tracked
        for i in range(self.cube_count):
            self.logger.info(f"Releasing cube index #{i+1}...")
            pivot_180_degree()
            
            # Fire valve
            lgpio.gpio_write(self.handle, self.CYLINDER_PIN, 0)
            time.sleep(0.8) 
            
            # Retract valve
            lgpio.gpio_write(self.handle, self.CYLINDER_PIN, 1)
            pivot_180_degree()
            time.sleep(0.5) 
            
        self.cube_count = 0 
        return True

# ==================== TESTING EXECUTION ENVIRONMENT ====================
def main():
    # Initialize a clean temporary ROS2 node context just to get its logger
    rclpy.init()
    test_node = Node('pneumatic_tester_node')
    logger = test_node.get_logger()
    
    logger.info("Initializing Unit Test for PickPlaceMechanism...")
    mechanism = PickPlaceMechanism(logger)

    print("\n--- TEST 1: Simulate Finding a Fake Cube ---")
    mechanism.junction_action(success=True, cube_detection=False)
    time.sleep(1)

    print("\n--- TEST 2: Simulate Finding 3 Real Cubes ---")
    mechanism.junction_action(success=True, cube_detection=True) # Cube 1
    time.sleep(1)
    mechanism.junction_action(success=True, cube_detection=True) # Cube 2
    time.sleep(1)
    mechanism.junction_action(success=True, cube_detection=True) # Cube 3
    
    print(f"\nIs inventory maxed out? {mechanism.cube_pick_status()}")
    time.sleep(1)

    print("\n--- TEST 3: Simulate Placement Unloading Sequence ---")
    mechanism.place_cubes()

    # Cleanup hardware bindings
    lgpio.gpiochip_close(mechanism.handle)
    test_node.destroy_node()
    rclpy.shutdown()
    print("\nTest completed successfully!")

if __name__ == '__main__':
    main()