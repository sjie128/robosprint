import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from letter_detector.letter_detector import CameraSubscriberNode
from robot_bringup_pkg.robot_bringup_pkg import PickPlaceMechanism

class ZoneDropController(Node):
    def __init__(self):
        super().__init__('zone_drop_controller')
        
        # Initialize your pick & place setup
        self.mechanism = PickPlaceMechanism(self.get_logger())
        self.bridge = CvBridge()
        
        # Subscriptions
        self.ir_sub = self.create_subscription(String, '/serial_ir_data', self.ir_callback, 10)
        self.cam_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # Internal states
        self.current_frame = None
        self.is_at_drop_zone = False
        self.drop_executed = False

    def image_callback(self, msg):
        # Continuously keep track of the latest camera frame
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def ir_callback(self, msg):
        """
        Listens to data coming from Arduino. 
        Format expected: "IR:rawL,rawC,rawR,junctionLabel"
        """
        data_str = msg.data
        if not data_str.startswith("IR:"):
            return

        # Split the message strings
        parts = data_str.replace("IR:", "").split(",")
        if len(parts) < 4:
            return
            
        junction_label = parts[3].strip()

        # Trigger condition: IR sensor senses ALL WHITE
        if junction_label == "White" or junction_label == "ALL_WHITE":
            self.is_at_drop_zone = True
            if not self.drop_executed:
                self.process_drop_zone_logic()
        else:
            self.is_at_drop_zone = False
            self.drop_executed = False # Reset flag when moving away

    def check_slot_occupancy(self, frame):
        """
        Splits the camera view into Left, Center, and Right slot sectors
        and checks if a cube is physically present in them using color/contour sizing.
        """
        # Convert image frame to HSV color space for steady color thresholding
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Adjust these ranges based on your competition cube color (e.g., Red, Blue, etc.)
        lower_cube = np.array([0, 50, 50])
        upper_cube = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_cube, upper_cube)
        
        height, width = mask.shape
        slot_w = width // 3
        
        # Define 3 search columns (Left, Center, Right)
        slots = {
            'left': mask[:, 0:slot_w],
            'center': mask[:, slot_w:slot_w*2],
            'right': mask[:, slot_w*2:width]
        }
        
        occupancy_status = {}
        for slot_name, slot_region in slots.items():
            # Count how many colored pixels are in the region
            pixel_count = cv2.countNonZero(slot_region)
            # If color detection passes a threshold limit, the slot is blocked
            occupancy_status[slot_name] = pixel_count > 500 
            
        return occupancy_status

    def process_drop_zone_logic(self):
        if self.current_frame is None:
            self.get_logger().warn("Drop zone reached, but camera frames aren't ready yet!")
            return

        self.get_logger().info("All White sensed! Analyzing dropping zones...")
        slots = self.check_slot_occupancy(self.current_frame)
        
        dropped = False

        # empty place
        for slot in ['center', 'left', 'right']:
            if not slots[slot]:
                self.get_logger().info(f"Slot '{slot}' is empty! Driving there to drop cube.")
                self.navigate_to_slot(slot)
                
                # Run your exact mechanism function
                self.mechanism.place_cubes()
                dropped = True
                break

        # not place dy
        if not dropped:
            self.get_logger().warn("All slots are FULL! Initiating stacking sequence onto center cube.")
            self.navigate_to_slot('center')
            
            # Stacking adjustments (optional extra lift/piston push timing modification)
            self.mechanism.place_cubes()
            dropped = True

        self.drop_executed = True

    def navigate_to_slot(self, slot_target):
        """Add your navigation commands here to align the robot base chassis"""
        if slot_target == 'left':
            self.get_logger().info("Shifting base sideways Left...")
        elif slot_target == 'right':
            self.get_logger().info("Shifting base sideways Right...")
        else:
            self.get_logger().info("Aligned perfectly with Center slot.")

def main(args=None):
    rclpy.init(args=args)
    node = ZoneDropController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()