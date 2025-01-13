# program to deploy obstacle avoidance
import math
import rospy  # needed for ROS

from geometry_msgs.msg import Twist      # ROS Twist message
from nav_msgs.msg import Odometry        # ROS Odometry
from sensor_msgs.msg import LaserScan    # ROS laser scan message
from tf.transformations import euler_from_quaternion

# topics 
motionTopic = '/cmd_vel'  # Topic to control robot's movement (output topic)
poseTopic = '/odom' 
laserTopic = '/scan'
safeCmdVelTopic = '/safe_cmd_vel' # topic to receive velocity from CTNode

# Global variables
gLoc = [0, 0, 0]  # x, y, yaw pose of robot
gBumperLeft, gBumperRight = False, False  # left/right close
gVel_pub = None
gSafeCmd = Twist()  # Store the latest safe command

# Callback for the laser range data
def laserCallback(msg):
    '''Call back function for laser range data'''
    global gBumperLeft, gBumperRight
    
    gBumperLeft, gBumperRight = False, False
    numRays = len(msg.ranges)  # total num readings
    width = int(numRays / 4)  # left/right bumper 'window'
    tooClose = 0.5  # threshold for bumper to activate
    
    for i in range(0, len(msg.ranges)):
        # Rule out bad readings first
        if not math.isnan(msg.ranges[i]) and not math.isinf(msg.ranges[i]) and msg.ranges[i] > 0:
            # Check for anything close left and right
            if msg.ranges[i] < tooClose:
                if i in range(0, width + 1):
                    gBumperLeft = True
                elif i in range(numRays - width, numRays + 1):
                    gBumperRight = True
    return

# poseCallback
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

def inputCmdCallback(input_cmd):
    '''Callback for input velocity commands to handle obstacle avoidance'''
    global gSafeCmd
    gSafeCmd = input_cmd

def OANode():
    global gVel_pub

    # Initialize OANode
    rospy.init_node('OANode', anonymous=True)

    # Register as a ROS publisher for the single velocity topic
    gVel_pub = rospy.Publisher(motionTopic, Twist, queue_size=10)  # Publish to /cmd_vel

    # Subscribe to laser scan data for obstacle detection
    scan_sub = rospy.Subscriber(laserTopic, LaserScan, laserCallback)
    
    # Register as a subscriber for the pose topic
    pose_sub = rospy.Subscriber(poseTopic, Odometry, poseCallback)

    # Register safe velocity as subscriber to vel topic (using /safe_cmd_vel for input)
    safe_sub = rospy.Subscriber(safeCmdVelTopic, Twist, inputCmdCallback)

    rospy.loginfo("Obstacle Avoidance Node is running.")

    # Use a loop to keep the node alive and process callbacks
    rate = rospy.Rate(10)  # 10 Hz, adjust as needed
    while not rospy.is_shutdown():
        output_cmd = Twist()

        if gBumperLeft or gBumperRight:
            # If an obstacle is detected on the left or right, try to move around it
            if gBumperLeft:
                # If there's an obstacle on the left, turn right, move forward, then turn back
                output_cmd.angular.z = -0.5  # Turn right
                output_cmd.linear.x = 0.2  # Move forward slowly
                gVel_pub.publish(output_cmd)
                rospy.sleep(3)  # Move forward for 1 second (adjust time as necessary)

                # After moving forward, turn back to original direction
                output_cmd.angular.z = 0.5  # Turn back to the original direction
                gVel_pub.publish(output_cmd)
                rospy.sleep(2)  # Adjust time for turning back

            elif gBumperRight:
                # If there's an obstacle on the right, turn left, move forward, then turn back
                output_cmd.angular.z = 0.5  # Turn left
                output_cmd.linear.x = 0.2  # Move forward slowly
                gVel_pub.publish(output_cmd)
                rospy.sleep(3)  # Move forward for 1 second (adjust time as necessary)

                # After moving forward, turn back to original direction
                output_cmd.angular.z = -0.5  # Turn back to the original direction
                gVel_pub.publish(output_cmd)
                rospy.sleep(2)  # Adjust time for turning back
        
        elif gBumperLeft and gBumperRight:
            output_cmd.linear.x = -0.5  # Move backward
            output_cmd.angular.z = 0.0
            gVel_pub.publish(output_cmd)
            rospy.sleep(3)

        else:
            # No obstacle detected, follow the safe command velocity
            output_cmd = gSafeCmd

        # Publish the output command to move the robot
        gVel_pub.publish(output_cmd)

        rate.sleep()


def callback_shutdown():
    print("Shutting down")
    if rospy.is_shutdown():
        return
    pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub.publish(msg) 
    rospy.sleep(5)
    return

#-------------------------------MAIN program----------------------
if __name__ == '__main__':
    try:
        rospy.on_shutdown(callback_shutdown)
        OANode()
    except rospy.ROSInterruptException:
        pass
