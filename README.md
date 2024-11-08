# Real-Time Ball Trajectory Detection

## Overview
The Real-Time Ball Trajectory Detection project provides a ROS2-based framework for detecting, tracking, and predicting the movement of a ball in real time. This project utilizes Gazebo simulation for throwing the ball with controlled forces and computer vision techniques to monitor its motion. The system predicts the ball's future path using linear regression, making it applicable for sports analytics, robotic interaction, and automation.

## Features
- **Real-Time Detection**: Uses OpenCV and ROS2 to identify the ball in a camera feed through HSV color masking.
- **Trajectory Prediction**: Linear regression-based prediction of the ball's future positions.
- **Simulation in Gazebo**: The ball's movement is simulated within Gazebo, with random spawning and force-based motion.
- **Visualization**: Displays the real-time trajectory and predicted path of the ball.

## Applications
This project is useful for:
- Sports analytics: Tracking and predicting ball motion in various sports.
- Robotics: Enhancing robotic systems to interact with or intercept moving objects.
- Automation and control: Simulating and analyzing object motion in controlled environments.

## Installation
1. **Install ROS2**: Follow the official ROS2 installation guide for your OS.
2. **Install Gazebo**: The simulation environment requires Gazebo. Install Gazebo compatible with ROS2.
3. **Clone the Repository**:
    ```bash
    git clone https://github.com/yourusername/real-time-ball-trajectory-detection.git
    cd real-time-ball-trajectory-detection
    ```
4. **Install Dependencies**:
    ```bash
    pip install opencv-python numpy
    ```
5. **Build the Package**:
    ```bash
    colcon build
    ```

## Usage
1. **Run Gazebo Simulation**: Start Gazebo with the environment specified in the repository.
2. **Launch ROS2 Node**:
    ```bash
    ros2 launch ball_trajectory_detection start_simulation.launch.py
    ```
3. **Real-Time Visualization**: The detected ball and its predicted trajectory will display in an OpenCV window.

## Implementation Details
- **ROS2 Nodes**: The `BallTrajectoryPredictor` node handles ball detection and tracking.
- **Computer Vision**: Color-based detection in HSV space, combined with morphological operations to filter noise.
- **Trajectory Prediction**: Linear regression on past positions to estimate the future path of the ball.
- **Gazebo Control**: ROS2 services apply randomized forces for realistic simulation.

## Future Enhancements
- **Non-linear Trajectory Prediction**: Integrate polynomial or adaptive models for better prediction in curved paths.
- **Advanced Object Detection**: Implement YOLO or Faster R-CNN for robust detection under varying lighting.
- **Multi-Object Simulation**: Extend support for tracking multiple balls and integrate with multi-robot systems.

## License
MIT License.

## Contributors
- Tarush Sonawala, Abhishek Biswas  - Manipal Institute of Technology, Department of Mechatronics Engineering.

## Acknowledgments
This project was created under the guidance of **Asha C S** Ma'am as a mini-project for *Machine Vision and Image Processing* at the Department of Mechatronics Engineering, Manipal Institute of Technology.

