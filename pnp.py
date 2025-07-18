# Computer Vision and Robotic Arm Control System
# This script uses Intel RealSense camera for object detection and controls a robotic arm
# to perform pick-and-place operations based on detected object positions

import cv2  # OpenCV library for computer vision operations (image processing, contour detection)
import numpy as np  # Numerical computing library for array operations and mathematical calculations
import pyrealsense2 as rs  # Intel RealSense SDK for depth camera integration and 3D point cloud processing
import rospy  # ROS (Robot Operating System) Python client library for robot communication
from geometry_msgs.msg import PoseStamped  # ROS message type for 3D pose data (position + orientation)
# To automatically generate a requirements file (requirements.txt) from your imports,
# you can use the 'pipreqs tool. Install it with:
#   pip install pipreqs
# Then run in your project directory:
#   pipreqs .
# This will scan your imports and create a requirements.txt file.
# Note: Some ROS packages (like geometry_msgs) are not pip-installable and must be installed via ROS.
# For standard Python packages, pipreqs will handle them automatically.
from std_msgs.msg import String  # ROS message type for string data (gripper commands)

# Global variables to track current arm pose
current_arm_pose = None
pose_received = False

def arm_pose_callback(pose_msg):
    """
    Callback function for arm pose subscriber.
    Updates the global current_arm_pose variable with the latest pose data.
    
    Args:
        pose_msg: PoseStamped message containing current arm position and orientation
    """
    global current_arm_pose, pose_received
    current_arm_pose = pose_msg
    pose_received = True

def wait_for_pose_update(timeout=10.0):
    """
    Waits for a new pose update from the arm.
    
    Args:
        timeout: Maximum time to wait in seconds
        
    Returns:
        bool: True if pose was received, False if timeout occurred
    """
    global pose_received
    start_time = rospy.Time.now()
    pose_received = False
    
    while not pose_received and not rospy.is_shutdown():
        if (rospy.Time.now() - start_time).to_sec() > timeout:
            rospy.logwarn("Timeout waiting for pose update")
            return False
        rospy.sleep(0.1)
    
    return pose_received

def is_arm_at_target(target_x, target_y, target_z, tolerance=0.01):
    """
    Checks if the arm has reached the target position within the specified tolerance.
    
    Args:
        target_x, target_y, target_z: Target position coordinates
        tolerance: Position tolerance in meters (default: 1cm)
        
    Returns:
        bool: True if arm is at target position, False otherwise
    """
    global current_arm_pose
    
    if current_arm_pose is None:
        return False
    
    current_x = current_arm_pose.pose.position.x
    current_y = current_arm_pose.pose.position.y
    current_z = current_arm_pose.pose.position.z
    
    # Calculate Euclidean distance to target
    distance = np.sqrt((current_x - target_x)**2 + (current_y - target_y)**2 + (current_z - target_z)**2)
    
    return distance <= tolerance

def detect_object(frame):
    """
    Detects objects in a frame based on HSV color thresholds.
    This function uses color-based segmentation to identify objects of interest.
    
    Args:
        frame: Input BGR image frame from the RealSense camera
        
    Returns:
        tuple: (x, y) pixel coordinates of detected object center, or None if no object found
    """
    # Convert BGR color space to HSV (Hue, Saturation, Value) for better color detection
    # HSV is more robust for color-based object detection than RGB
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define color range for green object detection (adjust these values for your target object)
    # Green hue ranges from approximately 35 to 85 in HSV
    lower = np.array([35, 50, 50])   # Lower HSV threshold for green color
    upper = np.array([85, 255, 255]) # Upper HSV threshold for green color
    mask = cv2.inRange(hsv, lower, upper)

    # Find contours (outlines) of detected objects in the binary mask
    # RETR_EXTERNAL: Only retrieves external contours (ignores holes inside objects)
    # CHAIN_APPROX_SIMPLE: Compresses horizontal, vertical, and diagonal segments
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Find the largest contour (assumed to be the main object of interest)
        c = max(contours, key=cv2.contourArea)
        
        # Calculate moments of the contour to find its center (centroid)
        M = cv2.moments(c)
        
        # Check if the contour has a valid area (prevents division by zero)
        if M["m00"] > 0:
            # Calculate centroid coordinates using moment formulas
            cx = int(M["m10"] / M["m00"])  # x-coordinate of centroid
            cy = int(M["m01"] / M["m00"])  # y-coordinate of centroid
            return (cx, cy)
    return None

def get_3d_point(depth_frame, pixel, intrinsics):
    """
    Converts a 2D pixel coordinate to 3D world coordinates using depth information.
    
    Args:
        depth_frame: RealSense depth frame containing distance data
        pixel: (x, y) pixel coordinates in the image
        intrinsics: Camera intrinsic parameters (focal length, principal point, etc.)
        
    Returns:
        list: [x, y, z] coordinates in meters relative to camera
    """
    # Get the depth value at the specified pixel location
    depth = depth_frame.get_distance(pixel[0], pixel[1])
    # Use RealSense SDK to deproject 2D pixel to 3D point using camera intrinsics
    # This converts from image coordinates to real-world coordinates
    point = rs.rs2_deproject_pixel_to_point(intrinsics, [pixel[0], pixel[1]], depth)
    return point  # [x, y, z] in meters

def move_arm_to(pose_pub, x, y, z, orientation=[0,0,0,1], timeout=30.0):
    """
    Sends a pose command to move the robotic arm to a specific 3D position and blocks until it arrives.
    
    Args:
        pose_pub: ROS publisher for arm pose commands
        x, y, z: Target position coordinates in meters
        orientation: Quaternion orientation [x, y, z, w] (default: identity quaternion)
        timeout: Maximum time to wait for arm to reach target in seconds
        
    Returns:
        bool: True if arm reached target position, False if timeout or error occurred
    """
    global current_arm_pose
    
    # Create a PoseStamped message for the target pose
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()  # Current timestamp
    pose.header.frame_id = "base_link"  # Reference frame (robot base)
    
    # Set the target position
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    
    # Set the target orientation (quaternion format)
    pose.pose.orientation.x = orientation[0]
    pose.pose.orientation.y = orientation[1]
    pose.pose.orientation.z = orientation[2]
    pose.pose.orientation.w = orientation[3]
    
    # Publish the pose command to the arm controller
    pose_pub.publish(pose)
    rospy.loginfo(f"Moving arm to position: ({x:.3f}, {y:.3f}, {z:.3f})")
    
    # Wait for arm to reach target position
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        # Check if we've exceeded the timeout
        if (rospy.Time.now() - start_time).to_sec() > timeout:
            rospy.logwarn(f"Timeout waiting for arm to reach target position ({x:.3f}, {y:.3f}, {z:.3f})")
            return False
        
        # Check if arm has reached the target position
        if is_arm_at_target(x, y, z):
            rospy.loginfo(f"Arm reached target position ({x:.3f}, {y:.3f}, {z:.3f})")
            return True
        
        # Small delay to prevent excessive CPU usage
        rospy.sleep(0.1)
    
    return False

def control_gripper(gripper_pub, command):
    """
    Sends commands to control the robotic gripper (open/close).
    
    Args:
        gripper_pub: ROS publisher for gripper commands
        command: String command ("open" or "close")
    """
    # Publish the gripper command (open or "close") as a String message to the gripper topic
    gripper_pub.publish(String(command))
    # Sleep for 1 second to give the gripper time to actuate before continuing
    rospy.sleep(1)

def main():
    """
    Main function that orchestrates the complete pick-and-place operation.
    Initializes camera, ROS publishers/subscribers, and runs the main processing loop.
    """
    global current_arm_pose, pose_received
    
    # Initialize ROS node for robot communication
    rospy.init_node('vision_pick_place')
    
    # Create ROS publishers for sending commands to the robot
    pose_pub = rospy.Publisher('/arm/goal_pose', PoseStamped, queue_size=1) # Position commands
    gripper_pub = rospy.Publisher('/gripper/command', String, queue_size=1)  # Gripper commands
    
    # Create ROS subscriber to monitor current arm pose
    # Adjust the topic name based on your robot's configuration
    pose_sub = rospy.Subscriber('/arm/current_pose', PoseStamped, arm_pose_callback, queue_size=1)
    
    # Wait for initial pose data
    rospy.loginfo("Waiting for initial arm pose data...")
    if not wait_for_pose_update(timeout=5.0):
        rospy.logwarn("No initial pose data received. Continuing anyway...")
    else:
        rospy.loginfo("Initial arm pose received")

    # Initialize RealSense camera pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure camera streams:
    # - Color stream: 640x480 resolution, BGR8 format, 30 FPS
    # - Depth stream: 640x480 resolution, Z16 format (16bit depth), 30 FPS
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    # Start the camera pipeline with the configured streams
    pipeline.start(config)
    
    # Create alignment object to align depth frame to color frame
    # This ensures depth and color pixels correspond to the same real-world points
    align = rs.align(rs.stream.color)

    try:
        # Main processing loop - runs until ROS is shut down or 'q' is pressed
        while not rospy.is_shutdown():
            # Wait for and get frames from the camera
            frames = pipeline.wait_for_frames()
            
            # Align depth frame to color frame for accurate 3D mapping
            frames = align.process(frames)
            
            # Extract color and depth frames
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            # Skip iteration if either frame is invalid
            if not color_frame or not depth_frame:
                continue

            # Convert color frame to numpy array for OpenCV processing
            color_image = np.asanyarray(color_frame.get_data())
            
            # Detect object in the color image
            obj_pixel = detect_object(color_image)
            
            if obj_pixel:
                # Object detected - proceed with pick-and-place operation
                
                # Get camera intrinsic parameters for 3D point calculation
                intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
                
                # Convert 2D pixel coordinates to 3D world coordinates
                obj_point = get_3d_point(depth_frame, obj_pixel, intrinsics)
                x, y, z = obj_point[0], obj_point[1], obj_point[2]
                
                # Print detected object position for debugging
                print(f"Object at: {x:.3f}, {y:.3f}, {z:.3f}")

                # Execute pick-and-place sequence with error handling:
                
                # 1. Move above the object (10cm above)
                if not move_arm_to(pose_pub, x, y, z + 0.1):
                    rospy.logerr("Failed to move above object. Aborting pick-and-place operation.")
                    break
                
                # 2. Move down to grasp position
                if not move_arm_to(pose_pub, x, y, z):
                    rospy.logerr("Failed to move to grasp position. Aborting pick-and-place operation.")
                    break
                
                # 3. Close gripper to grasp the object
                control_gripper(gripper_pub, "close")
                rospy.loginfo("Gripper closed - object grasped")
                
                # 4. Lift object (15cm above original position)
                if not move_arm_to(pose_pub, x, y, z + 0.15):
                    rospy.logerr("Failed to lift object. Attempting to continue...")
                
                # 5. Move to place location (20cm offset in Y direction)
                if not move_arm_to(pose_pub, x, y + 0.2, z + 0.15):
                    rospy.logerr("Failed to move to place location. Aborting pick-and-place operation.")
                    break
                
                # 6. Lower to place position
                if not move_arm_to(pose_pub, x, y + 0.2, z):
                    rospy.logerr("Failed to lower to place position. Aborting pick-and-place operation.")
                    break
                
                # 7. Open gripper to release the object
                control_gripper(gripper_pub, "open")
                rospy.loginfo("Gripper opened - object released")
                
                # 8. Move arm back up
                if not move_arm_to(pose_pub, x, y + 0.2, z + 0.15):
                    rospy.logwarn("Failed to move arm back up, but operation completed successfully")
                
                # Print completion message and exit loop
                rospy.loginfo("Pick and place operation completed successfully!")
                break

            # Display the color image for debugging and monitoring
            cv2.imshow('Color', color_image)
            
            # Check for 'q' key press to quit the application
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        # Cleanup: stop camera pipeline and close OpenCV windows
        pipeline.stop()
        cv2.destroyAllWindows()

# Entry point: only run main() if this script is executed directly
if __name__ == "__main__":
    main()
