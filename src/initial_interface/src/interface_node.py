#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#dumb first interfacer with the tiago robot, pretty much only uses the commands via publisher
#pretty sure we could also create ros actions

def base_control():
   
    #base control uses Twist
    base_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    base_vel_msg = Twist()
    #only linear x and angular z controls the base movement
    base_vel_msg.linear.x = 0.5 
    base_vel_msg.angular.z = 0.5

    while not rospy.is_shutdown():
	base_vel_pub.publish(base_vel_msg)

def torso_control():
    #torso (and all other joints on the robot) uses JointTrajectory
    torso_control_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)

    torso_jt = JointTrajectory()
    torso_jt.joint_names = ['torso_lift_joint'] #simply naming the joints on the torso (only 1)

    torso_jtp = JointTrajectoryPoint()
    torso_jtp.positions = [1]
    torso_jtp.time_from_start = rospy.Duration(2.5)  #it was important to set a duration along with it, as i got synch errors
    torso_jt.points.append(torso_jtp)
    
    while not rospy.is_shutdown():
        torso_control_pub.publish(torso_jt)

def arm_control():
    arm_control_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

    arm_jt = JointTrajectory()
    arm_jt.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']

    arm_jtp = JointTrajectoryPoint()
    arm_jtp.positions = [0.2, 0.0, -1.5, 1.94, -1.57, -0.5, 0.0]
    arm_jtp.time_from_start = rospy.Duration(2.5)
    arm_jt.points.append(arm_jtp)
    
    while not rospy.is_shutdown():
        arm_control_pub.publish(arm_jt)


def head_control():
    head_control_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)

    head_jt = JointTrajectory()
    head_jt.joint_names = ['head_1_joint', 'head_2_joint']
    head_jtp = JointTrajectoryPoint()
    head_jtp.positions = [0, -0.075]
    head_jtp.time_from_start = rospy.Duration(2.5)
    head_jt.points.append(head_jtp)
    
    while not rospy.is_shutdown():
        head_control_pub.publish(head_jt)

def navigation():
    """
    very simple navigation interface
    REQUIRES move_base node to be started (didnt start with the simple gazebo sim, had to load a gazebo sim with rviz, unsure if the node is started on the physical)
    
    this example only uses the publisher for navigation
    Tiago can start navigating in 4 ways:
	-the publish method used in this example (very simple, probably not optimal)
	-/move_base ROS ACTION  This does essentailly the same as the publish method, but you also get feedback when you reach the goal or something gows wrong
	-poi_navigation_server/go_to_poi ROS ACTION   makes robot go to point of interest made in the Map Editor (read manual for more info)
	-pal_waypoint/navigation ROS ACTION Makes robot visit all points of interest of a given group or subset.
    """
    nav_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    
    nav_goal = PoseStamped()
    
    #headers so ros can keep track of the goal
    nav_goal.header.seq = 3
    nav_goal.header.stamp = rospy.Duration(1)
    nav_goal.header.frame_id = 'map'
    
    #goal points on the rviz map
    nav_goal.pose.position.x = 0
    nav_goal.pose.position.y = 1.0
    nav_goal.pose.position.z = 0

    #orientation on the rviz map, in quaternion
    nav_goal.pose.orientation.x = 0
    nav_goal.pose.orientation.y = 0
    nav_goal.pose.orientation.z = 1
    nav_goal.pose.orientation.w = 0.99

    #just publishing once didnt guarantee that the goal point would be read
    for x in range(5000):
        nav_goal_pub.publish(nav_goal)



def callback(image_data):
    cv_image= cv_bridge.imgmsg_to_cv2(image_data, "bgr8")

def camera():
    camera_sub = rospy.Subscriber('xtion/rgb/image_raw', Image)
    rospy.spin()



if __name__ == '__main__':
   rospy.init_node('test', anonymous=True)
   #base_control()
   #torso_control()
   #arm_control()
   #head_control()
   #navigation()'
   camera()
