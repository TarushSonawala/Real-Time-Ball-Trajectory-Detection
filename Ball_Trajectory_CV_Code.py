import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from collections import deque

class BallTrajectoryPredictor(Node):
    def __init__(self):
        super().__init__('ball_trajectory_predictor')
        
        # Set parameters
        self.image_topic = "/camera1/camera_sensor/image_raw/compressed"  # Change to your camera's topic if different
        self.bridge = CvBridge()
        
        # HSV color range for ball detection (adjust based on ball color)
        self.ball_color_lower = np.array([0, 120, 70])  # Example for red color
        self.ball_color_upper = np.array([10, 255, 255])
        
        # Position tracking
        self.positions = deque(maxlen=30)  # To store recent ball positions
        
        # Image subscriber
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        
        # Window setup for visualization
        cv2.namedWindow("Ball Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Ball Detection", 800, 600)
        
    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
        
        # Detect the ball and draw its trajectory
        self.detect_and_draw_trajectory(frame)

        # Display the frame
        cv2.imshow("Ball Detection", frame)
        cv2.waitKey(1)

    def detect_and_draw_trajectory(self, frame):
        # Convert BGR to HSV for color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create a mask for the specified color range
        mask = cv2.inRange(hsv, self.ball_color_lower, self.ball_color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Assume largest contour is the ball
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            
            # Proceed if radius is within a sensible range
            if radius > 10:
                # Calculate the centroid
                M = cv2.moments(largest_contour)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
                # Append to recent positions
                self.positions.appendleft(center)
                
                # Draw circle and centroid
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                
                # Draw trajectory lines
                for i in range(1, len(self.positions)):
                    if self.positions[i - 1] is None or self.positions[i] is None:
                        continue
                    # Draw the trajectory line
                    cv2.line(frame, self.positions[i - 1], self.positions[i], (255, 0, 0), 2)
                
                # Predict and draw future trajectory
                self.predict_future_trajectory(frame)

    def predict_future_trajectory(self, frame):
        # Ensure there are enough points to predict
        if len(self.positions) >= 5:
            # Fit line to recent positions using polynomial fitting
            pts = np.array(self.positions, dtype=np.float32)
            [vx, vy, x0, y0] = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
            
            # Project future points along this line
            for t in range(5, 100, 5):
                x_pred = int(x0 + vx * t)
                y_pred = int(y0 + vy * t)
                cv2.circle(frame, (x_pred, y_pred), 3, (0, 255, 0), -1)

def main(args=None):
    rclpy.init(args=args)
    ball_trajectory_predictor = BallTrajectoryPredictor()
    rclpy.spin(ball_trajectory_predictor)
    ball_trajectory_predictor.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
