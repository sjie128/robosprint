#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
import glob
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CubeDetector:
    """Detect red or blue cubes in the frame."""
    
    def __init__(self, logger=None):
        self.logger = logger
    
    def log(self, msg, level='info'):
        """Log message using ROS logger if available, else print."""
        if self.logger:
            if level == 'info':
                self.logger.info(msg)
            elif level == 'error':
                self.logger.error(msg)
        else:
            print(msg)
    
    def detect_cubes(self, frame):
        """
        Detect red or blue cubes in the frame.
        Returns: list with the largest (color, bbox) tuple, or empty list if none detected
        """
        cubes = []
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Red color range (H: 0-10 and 170-180, S: 100-255, V: 100-255)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        # Blue color range (H: 100-130, S: 100-255, V: 100-255)
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([130, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Apply morphological operations to filter noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        
        # Find contours for red cubes
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_red = None
        largest_red_area = 0
        for contour in contours_red:
            area = cv2.contourArea(contour)
            if area > largest_red_area:
                largest_red_area = area
                x, y, w, h = cv2.boundingRect(contour)
                largest_red = ('red', (x, y, w, h))
        
        # Find contours for blue cubes
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_blue = None
        largest_blue_area = 0
        for contour in contours_blue:
            area = cv2.contourArea(contour)
            if area > largest_blue_area:
                largest_blue_area = area
                x, y, w, h = cv2.boundingRect(contour)
                largest_blue = ('blue', (x, y, w, h))
        
        # Return only the largest cube overall
        if largest_red and largest_blue:
            if largest_red_area > largest_blue_area:
                cubes.append(largest_red)
            else:
                cubes.append(largest_blue)
        elif largest_red:
            cubes.append(largest_red)
        elif largest_blue:
            cubes.append(largest_blue)
        
        return cubes


class AlphabetDetector:
    """Alphabet detection logic (same as standalone version)."""
    
    def __init__(self, templates_folder='templates', logger=None):
        """Initialize the detector with processed templates."""
        self.templates = {}
        self.logger = logger
        self.load_templates(templates_folder)
        
    def log(self, msg, level='info'):
        """Log message using ROS logger if available, else print."""
        if self.logger:
            if level == 'info':
                self.logger.info(msg)
            elif level == 'error':
                self.logger.error(msg)
            elif level == 'warn':
                self.logger.warn(msg)
        else:
            print(msg)
        
    def resize_with_pad(self, img, size=50):
        """Resize image with padding to maintain aspect ratio."""
        if len(img.shape) == 3:
            h, w, _ = img.shape
        else:
            h, w = img.shape
        
        scale = size / max(h, w)
        resized = cv2.resize(img, (int(w * scale), int(h * scale)))
        
        top = (size - resized.shape[0]) // 2
        bottom = size - resized.shape[0] - top
        left = (size - resized.shape[1]) // 2
        right = size - resized.shape[1] - left
        
        return cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=0)

    def load_templates(self, templates_folder):
        """Load all processed template images."""
        self.log(f"📁 Loading templates from '{templates_folder}'...")
        
        if not os.path.exists(templates_folder):
            self.log(f"❌ Error: Templates folder '{templates_folder}' not found!", 'error')
            return
        
        template_files = sorted(glob.glob(f"{templates_folder}/*_processed.png"))
        
        if not template_files:
            self.log(f"❌ No processed templates found in '{templates_folder}'!", 'error')
            return
        
        for template_path in template_files:
            letter = os.path.basename(template_path).split('_')[0]
            template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
            if template is not None:
                self.templates[letter] = template
                self.log(f"✅ Loaded template: {letter}")
        
        self.log(f"🎉 Total templates loaded: {len(self.templates)}")

    def extract_letter_from_frame(self, frame):
        """Extract the letter region from the webcam frame."""
        # Convert to HSV and extract V channel (brightness)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        _, _, v_channel = cv2.split(hsv)
        
        # Apply Gaussian blur and threshold
        blurred = cv2.GaussianBlur(v_channel, (5, 5), 0)
        _, thresh_inv = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # Find contours
        contours, _ = cv2.findContours(thresh_inv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None, None
        
        # Get the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Ignore tiny noise
        if cv2.contourArea(largest_contour) < 200:
            return None, None, None
        
        x, y, w, h = cv2.boundingRect(largest_contour)
        letter_crop = thresh_inv[y:y+h, x:x+w]
        
        # Resize with padding
        letter_normalized = self.resize_with_pad(letter_crop, 50)
        
        return letter_normalized, (x, y, w, h), largest_contour

    def detect_alphabet(self, frame):
        """Detect which alphabet is in the frame."""
        letter_normalized, bbox, contour = self.extract_letter_from_frame(frame)
        
        if letter_normalized is None:
            return None, None, 0, {}, None
        
        best_match = None
        best_score = -1
        scores = {}
        
        # Compare with all templates using template matching
        for letter, template in self.templates.items():
            # Ensure template and normalized letter have same shape
            if template.shape != letter_normalized.shape:
                template = cv2.resize(template, (letter_normalized.shape[1], letter_normalized.shape[0]))
            
            # Use correlation coefficient matching
            result = cv2.matchTemplate(letter_normalized, template, cv2.TM_CCOEFF)
            score = result[0, 0]
            scores[letter] = score
            
            if score > best_score:
                best_score = score
                best_match = letter
        
        # Normalize score to 0-100 range for display
        confidence = max(0, min(100, best_score / 100))
        
        return best_match, bbox, confidence, scores, letter_normalized

    def create_debug_panel(self, letter_normalized, scores, best_match, confidence):
        """Create a debug panel showing matching scores and normalized letter."""
        panel_height = 300
        panel_width = 500
        panel = np.zeros((panel_height, panel_width, 3), dtype=np.uint8)
        
        # Display the normalized letter in the top-left
        if letter_normalized is not None:
            normalized_display = cv2.cvtColor(letter_normalized, cv2.COLOR_GRAY2BGR)
            normalized_display = cv2.resize(normalized_display, (100, 100))
            panel[10:110, 10:110] = normalized_display
            cv2.putText(panel, "Detected Letter", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        # Sort scores and display all matching results
        y_offset = 150
        cv2.putText(panel, "Matching Scores:", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        y_offset += 30
        
        if scores:
            sorted_scores = sorted(scores.items(), key=lambda x: x[1], reverse=True)
            for i, (letter, score) in enumerate(sorted_scores):
                # Normalize display score
                display_score = max(0, min(100, score / 100))
                color = (0, 255, 0) if letter == best_match else (200, 200, 200)
                text = f"{letter}: {display_score:.1f}%"
                cv2.putText(panel, text, (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
                y_offset += 22
        
        return panel


class AlphabetDetectorNode(Node):
    """ROS2 Node for Alphabet Detection."""
    
    def __init__(self):
        super().__init__('alphabet_detector_node')
        
        # Declare parameters
        self.declare_parameter('templates_folder', 'src/letter_detector/letter_detector/templates')
        self.declare_parameter('confidence_threshold', 5.0)
        self.declare_parameter('image_topic', '/image_raw')
        
        # Get parameters
        self.templates_folder = self.get_parameter('templates_folder').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.image_topic = self.get_parameter('image_topic').value
        
        # Initialize cv_bridge for ROS image conversion
        self.bridge = CvBridge()
        
        self.get_logger().info("🚀 Initializing Alphabet Detector ROS2 Node...")
        
        # Initialize detectors
        self.cube_detector = CubeDetector(self.get_logger())
        self.detector = AlphabetDetector(self.templates_folder, self.get_logger())
        
        if not self.detector.templates:
            self.get_logger().error("❌ No templates loaded!")
            return
        
        # Create publishers
        self.detection_pub = self.create_publisher(String, 'alphabet_detection/result', 10)
        
        # Subscribe to image topic
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.process_frame_callback,
            10
        )
        
        self.get_logger().info(f"🎥 Alphabet Detector Node started!")
        self.get_logger().info(f"📸 Subscribing to: '{self.image_topic}'")
        self.get_logger().info(f"📦 Publishing to: 'alphabet_detection/result'")

    def process_frame_callback(self, ros_image):
        """Process frame from image topic callback."""
        try:
            # Convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"❌ Could not convert image: {e}")
            return
        
        display_frame = frame.copy()
        
        # Detect cubes (only process the first/largest one)
        cubes = self.cube_detector.detect_cubes(frame)
        
        if not cubes:
            # No cubes detected
            try:
                cv2.putText(display_frame, "No Cubes Detected", (10, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
                cv2.putText(display_frame, "Show RED or BLUE cube", (10, display_frame.shape[0] - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.imshow('Alphabet Detector ROS2 - Press Q to Quit', display_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info("\n✅ User requested shutdown...")
                    raise KeyboardInterrupt()
            except cv2.error:
                pass
            return
        
        # Process only the first (largest) detected cube
        cube_color, (cx, cy, cw, ch) = cubes[0]
        
        # Crop the cube region with some padding
        padding = 10
        crop_x = max(0, cx - padding)
        crop_y = max(0, cy - padding)
        crop_w = min(frame.shape[1] - crop_x, cw + 2 * padding)
        crop_h = min(frame.shape[0] - crop_y, ch + 2 * padding)
        
        cube_region = frame[crop_y:crop_y+crop_h, crop_x:crop_x+crop_w]
        
        # Detect alphabet on top of the cube
        best_match, bbox, confidence, scores, letter_normalized = self.detector.detect_alphabet(cube_region)
        
        # Prepare detection message
        if best_match is not None and confidence > self.confidence_threshold:
            result_msg = String()
            result_msg.data = f"Cube: {cube_color.upper()} | Letter: {best_match} | Confidence: {confidence:.1f}%"
            self.detection_pub.publish(result_msg)
            self.get_logger().info(f"✅ Detected: {cube_color} cube with letter {best_match} ({confidence:.1f}%)")
        
        # Draw cube bounding box
        color_bgr = (0, 0, 255) if cube_color == 'red' else (255, 0, 0)  # Red in BGR, Blue in BGR
        cv2.rectangle(display_frame, (cx, cy), (cx + cw, cy + ch), color_bgr, 3)
        cv2.putText(display_frame, f"{cube_color.upper()} Cube", (cx, cy - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
        
        # Draw letter detection result inside the cube region
        if best_match is not None and confidence > self.confidence_threshold:
            # Calculate absolute position of detected letter
            if bbox is not None:
                bx, by, bw, bh = bbox
                abs_bx = crop_x + bx
                abs_by = crop_y + by
                cv2.rectangle(display_frame, (abs_bx, abs_by), (abs_bx + bw, abs_by + bh), (0, 255, 0), 2)
            
            label = f"Letter: {best_match} ({confidence:.1f}%)"
            cv2.putText(display_frame, label, (cx, cy + ch + 25), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Create debug panel
        debug_panel = self.detector.create_debug_panel(letter_normalized, scores, best_match, confidence)
        
        # Combine display frame and debug panel side by side
        frame_resized = cv2.resize(display_frame, (int(display_frame.shape[1] * 300 / display_frame.shape[0]), 300))
        if frame_resized.shape[1] < 400:
            padding_width = 400 - frame_resized.shape[1]
            frame_resized = cv2.copyMakeBorder(frame_resized, 0, 0, 0, padding_width, cv2.BORDER_CONSTANT, value=0)
        else:
            frame_resized = frame_resized[:, :400]
        
        combined = np.hstack([frame_resized, debug_panel])
        
        # Display instructions
        cv2.putText(combined, "Show RED or BLUE cube with letter on top", (10, combined.shape[0] - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Display
        try:
            cv2.imshow('Alphabet Detector ROS2 - Press Q to Quit', combined)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("\n✅ User requested shutdown...")
                raise KeyboardInterrupt()
        except cv2.error:
            pass


    def destroy_node(self):
        """Clean up resources."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    node = AlphabetDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
