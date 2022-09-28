#!/usr/bin/env python
import time

import rospy
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Image, JointState, LaserScan
from nav_msgs.msg import Odometry

import stretch_body.lift
import stretch_body.arm
import stretch_body.head
import stretch_body.base
import stretch_body.robot

import math


import cv2
from cv_bridge import CvBridge, CvBridgeError

import tf2_ros
import tf_conversions

class HelloFormant():
    def __init__(self):

        rospy.init_node('Formant_hello', anonymous=True, log_level=rospy.DEBUG)

        rospy.loginfo("Launching Node")

        self.bridge = CvBridge()

        self.lift = stretch_body.lift.Lift()
        self.arm  = stretch_body.arm.Arm()
        self.head = stretch_body.head.Head()
        self.base = stretch_body.base.Base()

        # self.lift.motor.disable_sync_mode()
        if not self.lift.startup():
            rospy.loginfo("Failed to start lift")
            exit() # failed to start lift!

        if not self.arm.startup():
            rospy.loginfo("Failed to start arm")
            exit()

        if not self.head.startup():
            rospy.loginfo("Failed to start head")
            exit()

        if not self.base.startup():
            rospy.loginfo("Failed to start base")
            exit()


        self.lift.home()
        self.arm.home()
        self.head.home()

        self.base_frame_id = 'base_link'
        self.odom_frame_id = 'odom'

        rospy.Subscriber("/stow_arm", Bool, self.handle_stow_arm)
        rospy.Subscriber("/lift_arm", Float64, self.handle_lift_arm)
        rospy.Subscriber("/extend_arm", Float64, self.handle_extend_arm)

        rospy.Subscriber("/tilt_head", Float64, self.handle_head_tilt)
        rospy.Subscriber("/pan_head", Float64, self.handle_head_pan)

        rospy.Subscriber("/joystick", Twist, self.handle_joystick)
        rospy.Subscriber("/stretch/cmd_vel", Twist, self.handle_joystick)

        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)

        self.image_pub = rospy.Publisher("/camera/color/image_raw_rotated",Image, queue_size=1)

        self.joints_pub = rospy.Publisher('/jointstate', JointState, queue_size=1)

        self.laser_pub = rospy.Publisher('/reduced_scan', LaserScan, queue_size=1)

        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)

        rospy.Subscriber("/scan", LaserScan, self.reduced_points_repub)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()


    def handle_stow_arm(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Stowing Arm")
        if (msg.data):
            self.move_lift_to(0.2)

    def handle_lift_arm(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Lifting Arm")
        self.move_lift_to(msg.data)

    def handle_extend_arm(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Extending Arm")
        self.move_extension_to(msg.data)     

    def handle_head_tilt(self, msg):
        self.tilt_head_to(msg.data)

    def handle_head_pan(self, msg):
        self.pan_head_to(msg.data)

    def handle_joystick(self, msg):
        # command the base with translational and rotational velocities
        self.base.set_velocity(msg.linear.x, msg.angular.z)
        self.base.push_command()

        if msg.linear.y != 0 or msg.linear.z != 0:
            self.head.pull_status()
            pan_starting_position = self.head.status['head_pan']['pos']
            tilt_starting_position = self.head.status['head_tilt']['pos']

            self.head.move_to('head_pan',  pan_starting_position + 0.1*msg.linear.y)
            self.head.move_to('head_tilt', tilt_starting_position + 0.15*msg.linear.z)


    def move_extension_to(self,height):
        self.arm.move_to(height)
        self.arm.push_command()
        self.arm.motor.wait_until_at_setpoint()

    def move_lift_to(self,height):
        self.lift.move_to(height)
        self.lift.push_command()
        self.lift.motor.wait_until_at_setpoint()

    def pan_head_to(self,value):
        rospy.loginfo("Pan head")
        self.head.move_to('head_pan',value)
        self.head.push_command()
        # self.head.motor.wait_until_at_setpoint()

    def tilt_head_to(self,value):
        rospy.loginfo("Tilt head")
        self.head.move_to('head_tilt',value)
        self.head.push_command()
        # self.head.motor.wait_until_at_setpoint()


    def rgb_image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        image = cv2.transpose(image)
        image = cv2.flip(image,1)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "rgb8"))


    def get_joints(self):

        joints = JointState()

        joints.name = ["joint_lift", "joint_head_pan", "joint_head_tilt", "joint_arm_l4", "joint_arm_l3", "joint_arm_l2", "joint_arm_l1", "joint_arm_l0"]
        
        self.head.pull_status()
        pan_position = self.head.status['head_pan']['pos']
        tilt_position = self.head.status['head_tilt']['pos']

        self.lift.pull_status()
        lift_position = self.lift.status['pos']

        self.arm.pull_status()
        extention_length = self.arm.status['pos']
        l4_pos = extention_length / 5
        l3_pos = extention_length / 5
        l2_pos = extention_length / 5
        l1_pos = extention_length / 5
        l0_pos = extention_length / 5
        
        joints.position = [lift_position, pan_position, tilt_position, l4_pos, l3_pos, l2_pos, l1_pos, l0_pos]

        self.joints_pub.publish(joints)

    def reduced_points_repub(self, msg):
        if msg.header.seq % 8 == 0:
            msg.angle_min += math.pi
            msg.angle_max += math.pi
            self.laser_pub.publish(msg)

    def publish_odom_broadcast(self):

        current_time = rospy.Time.now()

        self.base.pull_status()
        base_status = self.base.status

        # obtain odometry
        x = base_status['x']
        y = base_status['y']
        theta = base_status['theta']
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        x_vel = base_status['x_vel']
        x_effort = base_status['effort'][0]
        theta_vel = base_status['theta_vel']
        pose_time_s = base_status['pose_time_s']

        # publish odometry via TF
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # publish odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = x_vel
        odom.twist.twist.angular.z = theta_vel
        self.odom_pub.publish(odom)

if __name__ == "__main__":

    node = HelloFormant()

    rate = rospy.Rate(10) # 10hz
    while True:
        # time.sleep(0.1)
        node.get_joints()
        node.publish_odom_broadcast()
        rate.sleep()
        