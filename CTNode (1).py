##Program that uses robot camera to track a colored target

import rospy
import cv2
import numpy as np
import time

from geomectry_msgs.msg import Twist      # ROS Twist message
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion

# ROS topic names for turtlebot_gazebo

#motionTopic ='/cmd_vel'  # Publish to safe_cmd_vel for OA node to subscribe
imageTopic = '/camera/rgb/image_raw'
poseTopic = '/odom'

# Global variables

gCurrentImage = Image() # make a global variable for the image
gBridge = CvBridge()    # make a ROS to CV bridge object
gImageStarted=False
color_detected = False
goal_number = 0
start_time = time.time()
gLoc = [0, 0, 0]  # x, y, yaw pose of robot

# List of color ranges for all targets with a visited attribute
color_ranges = {  # color range object for all targets
    'white': {'range': [(240, 240, 240), (255, 255, 255)], 'visited': False},  # Adjust these values
    'black': {'range': [(0, 0, 0), (20, 20, 20)], 'visited': False},
    'yellow': {'range': [(0, 80, 80), (10, 110, 110)], 'visited': False},  # Adjusted values for yellow
    'brown': {'range': [(0, 30, 75), (5, 50, 89)], 'visited': False},  # Adjusted values for brown
}

# topics 

#motionTopic='/safe_cmd_vel' 
poseTopic = '/odom' 
laserTopic  = '/scan'
safeMotionTopic = '/safe_cmd_vel'  # Safe command topic for obstacle avoidance

# Callback for the image topic
#
def callbackImage(img):
    '''Called automatically for each new image'''
    global gCurrentImage, gBridge, gImageStarted
    gCurrentImage = gBridge.imgmsg_to_cv2(img, "bgr8")
    gImageStarted=True
    print("Image received")
    return

# Callback for the pose topic
def poseCallback(data):
    '''Accept ROS pose topic info'''
    global gLoc
    gLoc[0] = data.pose.pose.position.x
    gLoc[1] = data.pose.pose.position.y
    orient = data.pose.pose.orientation
    quat = [orient.x, orient.y, orient.z, orient.w]
    (roll, pitch, yaw) = euler_from_quaternion(quat)
    gLoc[2] = yaw
    return

# Function to mark a color as visited
def mark_color_as_visited(color_name):
    if color_name in color_ranges:
        color_ranges[color_name]['visited'] = True

# Function to check if all colors have been visited
def all_colors_visited(color_ranges):
    return all(color_data['visited'] for color_data in color_ranges.values())

def track_and_detect_colors(color_ranges):
    detected_color = None
    detected_binary_image = None 

    # Apply Gaussian blur to reduce noise
    blurred_image = cv2.GaussianBlur(gCurrentImage, (5, 5), 0)

    # Check for the presence of each color
    for color_name, color_data in color_ranges.items():
        if not color_data['visited']:
            color_range = color_data['range']
            binary_image = cv2.inRange(blurred_image, np.array(color_range[0]), np.array(color_range[1]))

            # Apply morphological operations to remove noise and shadows
            kernel = np.ones((5, 5), np.uint8)
            binary_image = cv2.erode(binary_image, kernel, iterations=1)
            binary_image = cv2.dilate(binary_image, kernel, iterations=1)

            cv2.imshow(f'Target - {color_name}', cv2.resize(binary_image, (320, 240)))

            # Check if the color is detected and meets the area requirement
            m = cv2.moments(binary_image)
            area = m['m00']
            if area > 500000.0:  # Example area threshold
                print(f"Detected {color_name} with area {area}")
                detected_color = color_name
                detected_binary_image = binary_image
                break

    if detected_color is None:
        print("No unvisited colors detected.")
    return detected_color, detected_binary_image

def save_goal_image(goal_number, x, y, elapsed_time):
    '''Save the current image with goal information'''
    global gCurrentImage
    image_with_info = gCurrentImage.copy()
    text = f"Goal: {goal_number}, X: {x:.2f}, Y: {y:.2f}, Time: {elapsed_time:.2f}s"
    cv2.putText(image_with_info, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    filename = f"goal{goal_number}.jpg"
    cv2.imwrite(filename, image_with_info)
    print(f"Saved {filename}")

def CTnode(color_ranges):
    global goal_number, start_time
    rospy.init_node('CTnode', anonymous=True)
    imageSub = rospy.Subscriber(imageTopic, Image, callbackImage)
    poseSub = rospy.Subscriber(poseTopic, Odometry, poseCallback)
    vel_pub = rospy.Publisher(safeMotionTopic, Twist, queue_size=0)
    rospy.sleep(2)  # wait for callbacks to catch up

    # create windows to show camera and processing
    cv2.namedWindow('Turtlebot Camera', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('Target', cv2.WINDOW_AUTOSIZE)

    while not gImageStarted:
        rospy.sleep(1)  # wait for callback to catch up

    rate = rospy.Rate(20)  # Increase the rate to 20 Hz
    msg = Twist()

    while not rospy.is_shutdown():
        rospy.loginfo("Color Tracking Node is running.")

        if all_colors_visited(color_ranges):
            print("All colors have been visited.")
            break

        cv2.imshow('Turtlebot Camera', cv2.resize(gCurrentImage, (320, 240)))

        detected_color, detected_binary_image = track_and_detect_colors(color_ranges)

        if detected_color is None:
            # Rotate the robot if no color is detected
            msg.angular.z = 0.2
            msg.linear.x = 0.0
            vel_pub.publish(msg)
            cv2.waitKey(1)
            rate.sleep()
            continue

        # Navigate to the detected color
        avel = 0.0
        lvel = 0.0
        h, w = gCurrentImage.shape[0], gCurrentImage.shape[1]
        m = cv2.moments(detected_binary_image)
        if m['m00'] > 0:
            delx = w / 2 - m['m10'] / m['m00']
            avel = 0.5 * delx / w
            print(f"Target center for {detected_color} = ({round(m['m10'] / m['m00'], 2)}, {round(m['m01'] / m['m00'], 2)})")
            print(f"Offset from image center = {round(delx, 2)} => avel = {round(avel, 2)}")

            # Calculate the area of the detected target
            area = m['m00']
            print(f"Area of detected target: {area}")

            # Adjust linear velocity based on the area (distance)
            if abs(delx) < 10:
                lvel = 0.2 if area < 100000000.0 else 0.0  # Adjusted threshold for distance
                if lvel == 0.0:
                    mark_color_as_visited(detected_color)
                    elapsed_time = time.time() - start_time
                    save_goal_image(goal_number, gLoc[0], gLoc[1], elapsed_time)
                    goal_number += 1
                    print(f"Visited {detected_color}")
                    detected_color = None  # Reset detected_color to continue searching for the next color
                    continue
        
        msg.linear.x, msg.angular.z = lvel, avel
        vel_pub.publish(msg)
        cv2.waitKey(1)
        rate.sleep()

def callback_shutdown():
    print("Shutting down")
    if rospy.is_shutdown():
        return
    pub = rospy.Publisher(safeMotionTopic, Twist, queue_size=1)
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub.publish(msg) 
    rospy.sleep(5)
    return

if __name__ == '__main__':
    # identify/center the RGB color range of the target
    try:
        rospy.on_shutdown(callback_shutdown)
        CTnode(color_ranges)
    except rospy.ROSInterruptException:
        pass