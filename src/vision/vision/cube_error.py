import rclpy
from rclpy.node import Node
from cube_detection.msg import CubeDetectionArray 

class CubeAlignerNode(Node):
    def __init__(self):
        super().__init__('cube_aligner_node')
        
        # Camera resolution configuration (change to match your camera)
        self.target_x = 320.0  
        self.target_y = 240.0
        self.threshold = 10.0 # Pixel tolerance
        
        # Subscribe to cube detection array topic
        self.detection_sub = self.create_subscription(
            CubeDetectionArray, 
            'alphabet_detection/result', 
            self.on_cube_detected, 
            10
        )
        
        self.get_logger().info("Cube Aligner Node started. Waiting for detections...")

    def on_cube_detected(self, msg):
        # 1. Check if the array actually contains any detected cubes
        if not msg.detections:  # Note: change 'detections' to match your specific .msg array field name
            self.get_logger().info("No cubes detected in the frame.")
            self.stop_robot()
            return

        # 2. Get the first/closest cube from the array list
        # (Alternatively, loop through msg.detections to find a specific alphabet cube)
        target_cube = msg.detections[0] 
        
        # 3. Extract the float32 coordinates
        cx = target_cube.center_x
        cy = target_cube.center_y
        
        self.get_logger().info(f"Detected Cube Center -> X: {cx:.2f}, Y: {cy:.2f}")
        
        # 4. Calculate error relative to frame alignment target
        error_x = cx - self.target_x
        error_y = cy - self.target_y
        
        # 5. Determine movement state
        if abs(error_x) <= self.threshold and abs(error_y) <= self.threshold:
            self.stop_robot()
            self.trigger_nema_grabber()
        else:
            self.move_robot_base(error_x, error_y)

    def move_robot_base(self, error_x, error_y):
        # Implement your P-control logic and Twist/Velocity publisher here
        kp = 0.01 
        vel_x = error_x * kp
        vel_y = error_y * kp
        self.get_logger().info(f"Aligning base... Command Velocities: X={vel_x:.3f}, Y={vel_y:.3f}")

    def stop_robot(self):
        # Send zero velocity command to your base
        pass

    def trigger_nema_grabber(self):
        # Send command or publish message to trigger your NEMA stepper motor
        self.get_logger().info("Target centered perfectly! Activating NEMA grabber...")

def main(args=None):
    rclpy.init(args=args)
    node = CubeAlignerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()