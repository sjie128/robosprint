#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import glob
import threading
import queue


class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('templates_folder', '/home/team3/ros2_ws/src/letter_detector/letter_detector/templates')
        self.declare_parameter('confidence_threshold', 0.6)
        
        # Get parameter values
        camera_topic = self.get_parameter('camera_topic').value
        templates_folder = self.get_parameter('templates_folder').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        # Initialize CV Bridge for ROS image conversion
        self.bridge = CvBridge()
        
        # Queue for debug display
        self.debug_frame_queue = queue.Queue(maxsize=1)
        self.current_detection = {"letter": None, "score": 0.0, "bbox": None}
        self.detection_lock = threading.Lock()
        
        # Load alphabet templates
        self.templates = {}
        self.load_templates(templates_folder)
        
        # Subscribe to camera topic
        self.camera_subscription = self.create_subscription(
            Image,
            camera_topic,
            self.camera_callback,
            10
        )
        
        # Publisher for detected alphabet
        self.result_publisher = self.create_publisher(
            String,
            'detected_alphabet',
            10
        )
        
        # Start debug display thread
        self.debug_thread = threading.Thread(target=self.debug_display_loop, daemon=True)
        self.debug_thread.start()
        
        self.get_logger().info(f"📷 Camera subscriber node started")
        self.get_logger().info(f"🔗 Listening on topic: {camera_topic}")
        self.get_logger().info(f"📁 Templates loaded from: {templates_folder}")
        self.get_logger().info(f"🖼️  Debug screen running (close window to exit)")


    def load_templates(self, templates_folder):
        """Load all alphabet template images from folder."""
        self.get_logger().info(f"📂 Loading templates...")
        
        if not os.path.exists(templates_folder):
            self.get_logger().error(f"❌ Templates folder not found: {templates_folder}")
            return
        
        template_files = glob.glob(f"{templates_folder}/*.*")
        if not template_files:
            self.get_logger().error(f"❌ No templates found in {templates_folder}")
            return
        
        for file_path in sorted(template_files):
            filename = os.path.basename(file_path)
            letter = filename.split('.')[0].upper()
            
            # Read the template image
            template_img = cv2.imread(file_path)
            if template_img is None:
                self.get_logger().warn(f"⚠️  Failed to load: {filename}")
                continue
            
            # Convert to grayscale for template matching
            gray = cv2.cvtColor(template_img, cv2.COLOR_BGR2GRAY)
            self.templates[letter] = gray
            self.get_logger().info(f"✅ Loaded: {letter}")
        
        self.get_logger().info(f"🎉 Total templates loaded: {len(self.templates)}")

    def camera_callback(self, msg):
        """Process incoming camera frames."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to grayscale for processing
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Extract the letter region from the image
            letter_region, bbox = self.extract_letter_region(gray_image)
            
            if letter_region is None:
                # No letter found, add to debug queue with message
                processed_frame = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
                cv2.putText(processed_frame, "No letter detected", (20, 40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                try:
                    self.debug_frame_queue.put_nowait(cv2.cvtColor(processed_frame, cv2.COLOR_BGR2GRAY))
                except queue.Full:
                    pass
                return
            
            # Find matching alphabet for the extracted letter
            best_match, best_score = self.match_alphabet(letter_region)
            
            # Store detection result for debug display
            with self.detection_lock:
                self.current_detection["letter"] = best_match
                self.current_detection["score"] = best_score
                self.current_detection["bbox"] = bbox
            
            # Process frame for debug display
            blurred = cv2.GaussianBlur(gray_image, (5, 5), 0)
            # Use THRESH_BINARY_INV with a lower threshold to catch both red and blue boxes
            _, thresh = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)
            
            # Draw bounding box on processed frame
            if bbox:
                x, y, w, h = bbox
                cv2.rectangle(thresh, (x, y), (x+w, y+h), 128, 2)
            
            # Send processed frame to debug display
            try:
                self.debug_frame_queue.put_nowait(thresh)
            except queue.Full:
                pass
            
            # Publish result if confidence is above threshold
            if best_match and best_score >= self.confidence_threshold:
                result_msg = String()
                result_msg.data = f"{best_match}:{best_score:.3f}"
                self.result_publisher.publish(result_msg)
                self.get_logger().info(f"✅ Detected: {best_match} (score: {best_score:.3f})")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing image: {str(e)}")


    def extract_letter_region(self, gray_image):
        """
        Extract the letter region from the image.
        Assumes letter is on a red/blue colored cube background.
        Returns: (extracted_letter, bounding_box) or (None, None) if no letter found
        """
        # Find the colored (red/blue) regions using threshold
        # Invert so colored areas become black
        _, colored_areas = cv2.threshold(gray_image, 150, 255, cv2.THRESH_BINARY_INV)
        
        # Find contours of colored areas (the cube)
        contours, _ = cv2.findContours(colored_areas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return None, None
        
        # Get the largest colored region (should be the cube)
        largest_colored_contour = max(contours, key=cv2.contourArea)
        
        # Get bounding box of the cube
        x_cube, y_cube, w_cube, h_cube = cv2.boundingRect(largest_colored_contour)
        
        # Extract region of interest (the cube area)
        cube_region = gray_image[y_cube:y_cube+h_cube, x_cube:x_cube+w_cube]
        
        # Now find the letter within this cube region
        # Apply Gaussian blur for smoothing
        blurred = cv2.GaussianBlur(cube_region, (5, 5), 0)
        
        # Threshold to find dark areas (the letter) on the colored background
        _, thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY_INV)
        
        # Find contours of the letter with high precision
        contours_letter, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours_letter) == 0:
            return None, None
        
        # Get the largest dark contour (the letter)
        largest_letter_contour = max(contours_letter, key=cv2.contourArea)
        
        # Ignore tiny noise
        if cv2.contourArea(largest_letter_contour) < 50:
            return None, None
        
        # Get tight bounding box of the letter using contour
        x_letter, y_letter, w_letter, h_letter = cv2.boundingRect(largest_letter_contour)
        
        # Add small padding around the letter for extraction (5 pixels)
        padding = 5
        x_padded = max(0, x_letter - padding)
        y_padded = max(0, y_letter - padding)
        w_padded = min(w_cube - x_padded, w_letter + padding * 2)
        h_padded = min(h_cube - y_padded, h_letter + padding * 2)
        
        # Extract letter region with padding
        letter_region = thresh[y_padded:y_padded+h_padded, x_padded:x_padded+w_padded]
        
        # Return letter region and bounding box in original image coordinates (without padding for display)
        bbox_original = (x_cube + x_letter, y_cube + y_letter, w_letter, h_letter)
        
        return letter_region, bbox_original


    def match_alphabet(self, gray_image):
        """
        Compare input image against all templates.
        Returns: (matched_letter, confidence_score)
        """
        if not self.templates:
            return None, 0.0
        
        best_match = None
        best_score = -1
        
        # Ignore if image is too small
        if gray_image.shape[0] < 5 or gray_image.shape[1] < 5:
            return None, 0.0
        
        # Test against each template
        for letter, template in self.templates.items():
            try:
                # Resize input to match template size for comparison
                h, w = gray_image.shape
                scale = 50 / max(h, w)
                resized = cv2.resize(gray_image, (int(w * scale), int(h * scale)))
                
                # Pad to 50x50
                top = (50 - resized.shape[0]) // 2
                bottom = 50 - resized.shape[0] - top
                left = (50 - resized.shape[1]) // 2
                right = 50 - resized.shape[1] - left
                
                padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=0)
                
                # Use template matching with correlation coefficient
                result = cv2.matchTemplate(padded, template, cv2.TM_CCOEFF_NORMED)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
                
                # Use the maximum score
                score = max_val
                
                if score > best_score:
                    best_score = score
                    best_match = letter
            except:
                continue
        
        return best_match, best_score


    def debug_display_loop(self):
        """Display processed camera frames with detection results in a window."""
        window_name = "📷 Letter Detector - Processed Video"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 400, 300)  # Set window to 400x300
        
        try:
            while True:
                try:
                    # Get latest processed frame from queue (non-blocking)
                    processed_frame = self.debug_frame_queue.get(timeout=1)
                except queue.Empty:
                    continue
                
                # Resize frame to fit smaller window
                resized_frame = cv2.resize(processed_frame, (400, 300))
                
                # Convert grayscale to BGR for colored text overlay
                display_frame = cv2.cvtColor(resized_frame, cv2.COLOR_GRAY2BGR)
                
                # Get current detection
                with self.detection_lock:
                    letter = self.current_detection["letter"]
                    score = self.current_detection["score"]
                
                # Add background for text
                cv2.rectangle(display_frame, (10, 10), (350, 100), (50, 50, 50), -1)
                
                # Add detection info
                if letter:
                    text_color = (0, 255, 0) if score >= self.confidence_threshold else (0, 165, 255)
                    cv2.putText(display_frame, f"Detected: {letter}", (20, 40), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.2, text_color, 2)
                    cv2.putText(display_frame, f"Score: {score:.3f}", (20, 80), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.9, text_color, 2)
                else:
                    cv2.putText(display_frame, "No match", (20, 40), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
                
                # Show frame
                cv2.imshow(window_name, display_frame)
                
                # Check for window close or ESC key
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    break
                
                # Check if window was closed
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
                    
        except Exception as e:
            self.get_logger().error(f"Debug display error: {str(e)}")
        finally:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    node = CameraSubscriberNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
