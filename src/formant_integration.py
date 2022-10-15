#!/usr/bin/env python3
import time

from nbformat import current_nbformat_minor

import rospy
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import Twist, TransformStamped, Pose2D, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, JointState, LaserScan
from nav_msgs.msg import Odometry

import stretch_body.lift
import stretch_body.arm
import stretch_body.head
import stretch_body.base
import stretch_body.robot
import stretch_body.pimu
import stretch_body.robot

import math
import json

import cv2
from cv_bridge import CvBridge, CvBridgeError

import tf2_ros
import tf_conversions

from tf.transformations import euler_from_quaternion

class HelloFormant():
    def __init__(self):

        rospy.init_node('Formant_hello', anonymous=True, log_level=rospy.DEBUG)

        rospy.loginfo("Launching Node")

        self.bridge = CvBridge()

        self.robot = stretch_body.robot.Robot()

        self.lift = self.robot.lift
        self.arm  = self.robot.arm
        self.head = self.robot.head
        self.base = self.robot.base
        self.pimu = self.robot.pimu
        self.gripper = self.robot.end_of_arm

        self.x = 0
        self.y = 0
        self.theta = 0
        self.q = None

        self.laser_x = 0
        self.laser_y = 0
        self.laser_theta = 0

        self.runstop_reset()

        self.robot.startup()

        self.lift.home()
        self.arm.home()
        self.head.home()

        self.gripper.home("wrist_yaw")
        self.gripper.home("stretch_gripper")

        self.base_frame_id = 'base_link'
        self.odom_frame_id = 'odom'

        self.odom_amcl_pub = rospy.Publisher('/odom_amcl', Odometry, queue_size=1)
        
        rospy.Subscriber("/stow_arm", Bool, self.handle_stow_arm)

        rospy.Subscriber("/raise_to_top", Bool, self.handle_raise_to_top)
        rospy.Subscriber("/lower_to_bottom", Bool, self.handle_lower_to_bottom)
        rospy.Subscriber("/open_gripper", Bool, self.handle_open_gripper)
        rospy.Subscriber("/close_gripper", Bool, self.handle_close_gripper)

        rospy.Subscriber("/lift_arm", Float64, self.handle_lift_arm)
        rospy.Subscriber("/extend_arm", Float64, self.handle_extend_arm)

        rospy.Subscriber("/tilt_head", Float64, self.handle_head_tilt)
        rospy.Subscriber("/pan_head", Float64, self.handle_head_pan)

        rospy.Subscriber("/joystick", Twist, self.handle_joystick)
        rospy.Subscriber("/stretch/cmd_vel", Twist, self.handle_joystick)

        rospy.Subscriber("/wrist_yaw", Float64, self.handle_wrist_yaw)
        rospy.Subscriber("/gripper", Float64, self.handle_gripper)

        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.handle_pose_amcl)
        rospy.Subscriber("/pose2D", Pose2D, self.handle_pose_lasermatch_odom)

        self.image_pub = rospy.Publisher("/camera/color/image_raw_rotated",Image, queue_size=1)

        self.joints_pub = rospy.Publisher('/jointstate', JointState, queue_size=1)

        self.laser_pub = rospy.Publisher('/reduced_scan', LaserScan, queue_size=1)
        self.laser_pub_map = rospy.Publisher('/reduced_scan_map', LaserScan, queue_size=1)

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)

        #self.odom_amcl_pub = rospy.Publisher('/odom_amcl', Odometry, queue_size=1)

        rospy.Subscriber("/scan", LaserScan, self.reduced_points_repub)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()


        self.voltage_pub = rospy.Publisher('stretch_voltage', Float64, queue_size=1)

        self.current_pub = rospy.Publisher('stretch_current', Float64, queue_size=1)
        self.temp_pub = rospy.Publisher('stretch_temperature', Float64, queue_size=1)
        self.cpu_temp_pub = rospy.Publisher('stretch_cpu_temperature', Float64, queue_size=1)

        self.imu_acc_x_pub = rospy.Publisher('stretch_imu_ax', Float64, queue_size=1)
        self.imu_acc_y_pub = rospy.Publisher('stretch_imu_ay', Float64, queue_size=1)
        self.imu_acc_z_pub = rospy.Publisher('stretch_imu_az', Float64, queue_size=1)
        self.imu_rot_x_pub = rospy.Publisher('stretch_imu_gx', Float64, queue_size=1)
        self.imu_rot_y_pub = rospy.Publisher('stretch_imu_gy', Float64, queue_size=1)
        self.imu_rot_z_pub = rospy.Publisher('stretch_imu_gz', Float64, queue_size=1)
        self.imu_mag_x_pub = rospy.Publisher('stretch_imu_mx', Float64, queue_size=1)
        self.imu_mag_y_pub = rospy.Publisher('stretch_imu_my', Float64, queue_size=1)
        self.imu_mag_z_pub = rospy.Publisher('stretch_imu_mz', Float64, queue_size=1)


        self.pimu_cliff_pub = rospy.Publisher('stretch_pimu_cliff_event', Bool, queue_size=1)
        self.pimu_buzzer_on_pub = rospy.Publisher('stretch_pimu_buzzer_on', Bool, queue_size=1)
        self.pimu_runstop_event_pub = rospy.Publisher('stretch_pimu_runstop', Bool, queue_size=1)
        self.pimu_fan_on_pub = rospy.Publisher('stretch_pimu_fan', Bool, queue_size=1)
        self.pimu_high_current_pub = rospy.Publisher('stretch_pimu_high_current', Bool, queue_size=1)
        self.pimu_over_tilt_pub = rospy.Publisher('stretch_pimu_over_tilt', Bool, queue_size=1)

        self.pimu_transport_pub = rospy.Publisher('stretch_pimu_transport', String, queue_size=1)


        self.arm_force_pub = rospy.Publisher('stretch_arm_force', Float64, queue_size=1)
        self.arm_pos_pub = rospy.Publisher('stretch_arm_pos', Float64, queue_size=1)
        self.lift_force_pub = rospy.Publisher('stretch_lift_force', Float64, queue_size=1)
        self.lift_pos_pub = rospy.Publisher('stretch_lift_pos', Float64, queue_size=1)
        self.head_tilt_pos_pub = rospy.Publisher('stretch_head_tilt_pos', Float64, queue_size=1)
        self.head_tilt_effort_pub = rospy.Publisher('stretch_head_tilt_effort', Float64, queue_size=1)
        self.head_pan_pos_pub = rospy.Publisher('stretch_head_pan_pos', Float64, queue_size=1)
        self.head_pan_effort_pub = rospy.Publisher('stretch_head_pan_effort', Float64, queue_size=1)
        self.base_right_wheel_pos_pub = rospy.Publisher('stretch_right_wheel_pos', Float64, queue_size=1)
        self.base_right_wheel_current_pub = rospy.Publisher('stretch_right_wheel_current', Float64, queue_size=1)
        self.left_wheel_pos_pub = rospy.Publisher('stretch_left_wheel_pos', Float64, queue_size=1)
        self.left_wheel_current_pub = rospy.Publisher('stretch_left_wheel_current', Float64, queue_size=1)


        self.lift_state_pub = rospy.Publisher('/lift_state', Float64, queue_size=1)
        self.extend_state_pub = rospy.Publisher('/extend_state', Float64, queue_size=1)
        self.pan_head_state_pub = rospy.Publisher('/pan_head_state', Float64, queue_size=1)
        self.tilt_head_state_pub = rospy.Publisher('/tilt_head_state', Float64, queue_size=1)
        self.gripper_state_pub = rospy.Publisher('/gripper_state', Float64, queue_size=1)
        self.wrist_yaw_state_pub = rospy.Publisher('/wrist_yaw_state', Float64, queue_size=1)

        self.disable_amcl=rospy.get_param('disable_amcl_broadcast')

    def handle_pose_lasermatch(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

        self.publish_odom_laser_broadcast()

    def handle_pose_lasermatch_odom(self, msg):
        self.laser_x = msg.x
        self.laser_y = msg.y
        self.laser_theta = msg.theta

        self.publish_odom_broadcast(True)        

    def handle_pose_amcl(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        self.q = [quat.x, quat.y, quat.z, quat.w]
        
        (r, p , y) = euler_from_quaternion(self.q) 
        self.theta = y
        
        print(str(self.x) + " " + str(self.y) + " " + str(self.theta ))

        self.publish_odom_amcl_broadcast()

    def handle_stow_arm(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Stowing Arm")
        if (msg.data):
            self.move_lift_to(0.2)


    def handle_raise_to_top(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Raising Arm")
        if (msg.data):
            self.move_lift_to(1.099)

    def handle_lower_to_bottom(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Lowering Arm")
        if (msg.data):
            self.move_lift_to(0.1)

    def handle_open_gripper(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Open Gripper")
        if (msg.data):
            self.move_gripper_to(0)

    def handle_close_gripper(self, msg):
        rospy.loginfo(msg)
        rospy.loginfo("Close Gripper")
        if (msg.data):
            self.move_gripper_to(-50)                                    

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

    def handle_wrist_yaw(self, msg):
        self.move_wrist_to(msg.data)

    def handle_gripper(self, msg):
        self.move_gripper_to(msg.data)

    def handle_joystick(self, msg):
        # command the base with translational and rotational velocities

        if (msg.linear.x < 0):
            msg.angular.z = -msg.angular.z

        self.base.set_velocity(msg.linear.x, msg.angular.z)
        self.robot.push_command()

        # print(str(msg.linear.x) + " " + str(msg.angular.z))

        if msg.linear.y != 0 or msg.linear.z != 0:
            self.head.pull_status()
            pan_starting_position = self.head.status['head_pan']['pos']
            tilt_starting_position = self.head.status['head_tilt']['pos']

            self.head.move_by('head_pan', 0.3*msg.linear.y)
            self.head.move_by('head_tilt', 0.3*msg.linear.z)


    def move_extension_to(self,height):
        self.arm.move_to(height)
        self.robot.push_command()
        # self.arm.motor.wait_until_at_setpoint()

    def move_lift_to(self,height):
        self.lift.move_to(height)
        self.robot.push_command()
        # self.lift.motor.wait_until_at_setpoint()

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


    def move_wrist_to(self, value):
        rospy.loginfo("Wrist Yaw")
        self.gripper.move_to('wrist_yaw', value)
        self.gripper.push_command()

    def move_gripper_to(self, value):
        rospy.loginfo("Gripper")
        self.gripper.move_to('stretch_gripper', value)
        self.gripper.push_command()

    def rgb_image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        image = cv2.transpose(image)
        image = cv2.flip(image,1)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "rgb8"))


    def runstop_stop(self):
        self.pimu.runstop_event_trigger()
        self.pimu.push_command()
        time.sleep(1)

    def runstop_reset(self):
        self.pimu.runstop_event_reset()
        self.pimu.push_command()
        time.sleep(1)


    def get_joints(self):

        joints = JointState()

        joints.name = ["joint_lift", "joint_head_pan", "joint_head_tilt", "joint_arm_l4", "joint_arm_l3", "joint_arm_l2", "joint_arm_l1", "joint_arm_l0", "joint_wrist_yaw", "joint_gripper_finger_left", "joint_gripper_finger_right"]
        
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

        self.gripper.pull_status()
        wrist_yaw = self.gripper.status["wrist_yaw"]["pos"]
        gripper_l = self.gripper.status["stretch_gripper"]["pos"]*0.1 + 0.4
        gripper_r = gripper_l
        
        joints.position = [lift_position, pan_position, tilt_position, l4_pos, l3_pos, l2_pos, l1_pos, l0_pos, wrist_yaw, gripper_l, gripper_r]

        self.joints_pub.publish(joints)


        self.lift_state_pub.publish(lift_position)
        self.extend_state_pub.publish(extention_length)
        self.pan_head_state_pub.publish(pan_position)
        self.tilt_head_state_pub.publish(tilt_position)
        self.gripper_state_pub.publish( self.gripper.status["stretch_gripper"]["pos"]*50/4.72619480748 )
        self.wrist_yaw_state_pub.publish(wrist_yaw)

    def reduced_points_repub(self, msg):
        if msg.header.seq % 8 == 0:
            msg.angle_min += math.pi
            msg.angle_max += math.pi
            msg.header.frame_id = "base_link"
            self.laser_pub.publish(msg)

            msg.header.frame_id = "map"
            self.laser_pub_map.publish(msg)

    def publish_odom_broadcast(self, use_laser=False):

        current_time = rospy.Time.now()

        self.base.pull_status()
        base_status = self.base.status

        # obtain odometry
        x = base_status['x']
        y = base_status['y']
        theta = base_status['theta']
        if use_laser:
            x = self.laser_x
            y = self.laser_y
            theta = self.laser_theta
        # x = self.x
        # y = self.y
        # theta = self.theta
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



    def publish_odom_laser_broadcast(self):

        current_time = rospy.Time.now()

        self.base.pull_status()
        base_status = self.base.status

        # obtain odometry
        x = self.x
        y = self.y
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta)
        x_vel = base_status['x_vel']
        x_effort = base_status['effort'][0]
        theta_vel = base_status['theta_vel']
        pose_time_s = base_status['pose_time_s']

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
        self.odom_amcl_pub.publish(odom)

    def publish_odom_amcl_broadcast(self):

        current_time = rospy.Time.now()

        self.base.pull_status()
        base_status = self.base.status

        # obtain odometry
        x = self.x
        y = self.y
        q = self.q
        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        x_vel = base_status['x_vel']
        x_effort = base_status['effort'][0]
        theta_vel = base_status['theta_vel']
        pose_time_s = base_status['pose_time_s']

        # publish odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "map"
        odom.child_frame_id = "odom"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = x_vel
        odom.twist.twist.angular.z = theta_vel
        self.odom_amcl_pub.publish(odom)

    def publish_pimu_status(self):
        self.pimu.pull_status()

        self.publish_float64_msg(self.pimu.status['voltage'], self.voltage_pub)
        self.publish_float64_msg(self.pimu.status['current'], self.current_pub)
        self.publish_float64_msg(self.pimu.status['temp'], self.temp_pub)
        self.publish_float64_msg(self.pimu.status['cpu_temp'], self.cpu_temp_pub)

        self.publish_float64_msg(self.pimu.status['imu']['ax'], self.imu_acc_x_pub)
        self.publish_float64_msg(self.pimu.status['imu']['ay'], self.imu_acc_y_pub)
        self.publish_float64_msg(self.pimu.status['imu']['az'], self.imu_acc_z_pub)
        self.publish_float64_msg(self.pimu.status['imu']['gx'], self.imu_rot_x_pub)
        self.publish_float64_msg(self.pimu.status['imu']['gy'], self.imu_rot_y_pub)
        self.publish_float64_msg(self.pimu.status['imu']['gz'], self.imu_rot_z_pub)
        self.publish_float64_msg(self.pimu.status['imu']['mx'], self.imu_mag_x_pub)
        self.publish_float64_msg(self.pimu.status['imu']['my'], self.imu_mag_y_pub)
        self.publish_float64_msg(self.pimu.status['imu']['mz'], self.imu_mag_z_pub)

        self.publish_bool_msg(self.pimu.status['cliff_event'], self.pimu_cliff_pub)
        self.publish_bool_msg(self.pimu.status['buzzer_on'], self.pimu_buzzer_on_pub)
        self.publish_bool_msg(self.pimu.status['runstop_event'], self.pimu_runstop_event_pub)
        self.publish_bool_msg(self.pimu.status['fan_on'], self.pimu_fan_on_pub)
        self.publish_bool_msg(self.pimu.status['high_current_alert'], self.pimu_high_current_pub)
        self.publish_bool_msg(self.pimu.status['over_tilt_alert'], self.pimu_over_tilt_pub)

        self.publish_string_msg(json.dumps(self.pimu.status['transport']), self.pimu_transport_pub)

        # {'low_voltage_alert': False, 
        # 'cliff_range': [-40.50958251953125, -21.00262451171875, -20.82708740234375, -0.48065185546875], 
        # 'transport': {'transaction_time_max': 0, 'timestamp_pc': 0, 'read_error': 0, 'transaction_time_avg': 0, 'rate': 203.57734310537302, 'write_error': 0, 'itr': 3}, 
        # 'cliff_event': False, 
        # 'state': 0, 
        # 'buzzer_on': False, 
        # 'runstop_event': False, 
        # 'timestamp_status_sync': 0.000000, 
        # 'timestamp': 145.991137, 
        # 'fan_on': False, 
        # 'frame_id': 0, 
        # 'timestamp_pc': 1664427738.88845, 
        # 'timestamp_line_sync': 145.998026, 
        # 'bump_event_cnt': 0, 
        # 'at_cliff': [False, False, False, False], 
        # 'high_current_alert': False, 
        # 'debug': 1923.0, 
        # 'over_tilt_alert': False}


    def publish_body_statuses(self):

        self.arm.pull_status()
        self.publish_float64_msg(self.arm.status["force"], self.arm_force_pub)
        self.publish_float64_msg(self.arm.status["pos"], self.arm_pos_pub)

        self.lift.pull_status()
        self.publish_float64_msg(self.lift.status["force"], self.lift_force_pub)
        self.publish_float64_msg(self.lift.status["pos"], self.lift_pos_pub)

        self.head.pull_status()
        self.publish_float64_msg(self.head.status["head_tilt"]["pos"], self.head_tilt_pos_pub)
        self.publish_float64_msg(self.head.status["head_tilt"]["effort"], self.head_tilt_effort_pub)
        self.publish_float64_msg(self.head.status["head_pan"]["pos"], self.head_pan_pos_pub)
        self.publish_float64_msg(self.head.status["head_pan"]["effort"], self.head_pan_effort_pub)

        self.base.pull_status()
        self.publish_float64_msg(self.base.status["right_wheel"]["pos"], self.base_right_wheel_pos_pub)
        self.publish_float64_msg(self.base.status["right_wheel"]["current"], self.base_right_wheel_current_pub)
        self.publish_float64_msg(self.base.status["left_wheel"]["pos"], self.left_wheel_pos_pub)
        self.publish_float64_msg(self.base.status["left_wheel"]["current"], self.left_wheel_current_pub)
        
        ## Other available information
        # {'force': 2.956118405342102, 'timestamp_pc': 1664430119.843408, 'pos': 0.09999768526774287, 'traj_error': -0.09999768526774287, 'motor': {'in_sync_mode': False, 'is_mg_accelerating': False, 'calibration_rcvd': True, 'pos': 14.578909873962402, 'near_vel_setpoint': False, 'pos_calibrated': True, 'at_current_limit': False, 'current': 0.05288226127624512, 'in_guarded_event': False, 'transport': {'transaction_time_max': 0, 'timestamp_pc': 0, 'read_error': 0, 'transaction_time_avg': 0, 'rate': 10.044095989118487, 'write_error': 0, 'itr': 173}, 'vel': -0.026414502412080765, 'pos_traj': 0.0, 'timestamp': 8721.911023, 'runstop_on': False, 'guarded_event': 10, 'trajectory_active': False, 'is_mg_moving': False, 'in_safety_event': False, 'effort': 4.40685510635376, 'is_moving': False, 'err': -1.1717908819264267e-05, 'timestamp_line_sync': 0.000000, 'near_pos_setpoint': True, 'mode': 5, 'diag': 261, 'debug': 0.0, 'waiting_on_sync': False}, 'traj_err': 0.0, 'vel': -0.00018117877958932633}
        # {'right_wheel': {'in_sync_mode': False, 'is_mg_accelerating': False, 'calibration_rcvd': True, 'pos': 3080.986572265625, 'near_vel_setpoint': False, 'pos_calibrated': False, 'at_current_limit': False, 'current': -6.726232628759122e-46, 'in_guarded_event': False, 'transport': {'transaction_time_max': 0, 'timestamp_pc': 0, 'read_error': 0, 'transaction_time_avg': 0, 'rate': 10.117556813658918, 'write_error': 0, 'itr': 82}, 'vel': 0.1115519106388092, 'pos_traj': 0.0, 'timestamp': 8726.051032, 'runstop_on': False, 'guarded_event': 0, 'trajectory_active': False, 'is_mg_moving': False, 'in_safety_event': False, 'effort': -5.605193857299268e-44, 'is_moving': False, 'err': 0.0, 'timestamp_line_sync': 0.000000, 'near_pos_setpoint': False, 'mode': 0, 'diag': 256, 'debug': 0.0, 'waiting_on_sync': False}, 'left_wheel': {'in_sync_mode': False, 'is_mg_accelerating': False, 'calibration_rcvd': True, 'pos': 3717.80517578125, 'near_vel_setpoint': False, 'pos_calibrated': False, 'at_current_limit': False, 'current': -6.726232628759122e-46, 'in_guarded_event': False, 'transport': {'transaction_time_max': 0, 'timestamp_pc': 0, 'read_error': 0, 'transaction_time_avg': 0, 'rate': 10.12371079277344, 'write_error': 0, 'itr': 82}, 'vel': -0.07029969990253448, 'pos_traj': 0.0, 'timestamp': 8716.777032, 'runstop_on': False, 'guarded_event': 0, 'trajectory_active': False, 'is_mg_moving': False, 'in_safety_event': False, 'effort': -5.605193857299268e-44, 'is_moving': False, 'err': 0.0, 'timestamp_line_sync': 0.000000, 'near_pos_setpoint': False, 'mode': 0, 'diag': 256, 'debug': 0.0, 'waiting_on_sync': False}, 'y_vel': 0.0, 'x': 1.094324671920253e-05, 'effort': [0, 0], 'theta_vel': 0.0010516439115676828, 'pose_time_s': 7.927008999999998, 'rotation_torque': 0.0, 'timestamp_pc': 1664430119.848871, 'y': 4.853148299899246e-10, 'theta': 0.000138829616044082, 'translation_force': -2.849232141542364e-44, 'x_vel': 5.5263887576803554e-05}
        # {'force': -1.5717247009277346, 'timestamp_pc': 1664430119.840866, 'pos': 0.5930207017296297, 'traj_error': -0.5930207017296297, 'motor': {'in_sync_mode': False, 'is_mg_accelerating': False, 'calibration_rcvd': True, 'pos': 62.100982666015625, 'near_vel_setpoint': False, 'pos_calibrated': True, 'at_current_limit': False, 'current': 0.5190436706542969, 'in_guarded_event': False, 'transport': {'transaction_time_max': 0, 'timestamp_pc': 0, 'read_error': 0, 'transaction_time_avg': 0, 'rate': 10.018832319738584, 'write_error': 0, 'itr': 488}, 'vel': -0.008914763107895851, 'pos_traj': 0.0, 'timestamp': 8730.181023, 'runstop_on': False, 'guarded_event': 15, 'trajectory_active': False, 'is_mg_moving': False, 'in_safety_event': False, 'effort': 43.253639221191406, 'is_moving': False, 'err': -0.00017470336752012372, 'timestamp_line_sync': 0.000000, 'near_pos_setpoint': True, 'mode': 8, 'diag': 261, 'debug': 0.0, 'waiting_on_sync': False}, 'vel': -8.512971690689353e-05}
        # {'head_tilt': {'comm_errors': 0, 'vel_ticks': 0, 'overheating_error': False, 'temp': 41.0, 'timestamp_pc': 1664430119.79559, 'stalled': True, 'electrical_shock_error': False, 'pos': -0.01227184630308513, 'trajectory_active': False, 'stall_overload': False, 'overload_error': False, 'pos_ticks': 2056, 'hardware_error': 0, 'shutdown': 0, 'effort_ticks': -45, 'vel': -0.0, 'effort': -4.39453125, 'motor_encoder_error': False, 'input_voltage_error': False}, 'head_pan': {'comm_errors': 0, 'vel_ticks': 0, 'overheating_error': False, 'temp': 36.0, 'timestamp_pc': 1664430119.79559, 'stalled': True, 'electrical_shock_error': False, 'pos': -0.0015339807878856412, 'trajectory_active': False, 'stall_overload': False, 'overload_error': False, 'pos_ticks': 1166, 'hardware_error': 0, 'shutdown': 0, 'effort_ticks': -5, 'vel': -0.0, 'effort': -0.48828125, 'motor_encoder_error': False, 'input_voltage_error': False}}
        # {'force': 2.865717943954468, 'timestamp_pc': 1664430119.946876, 'pos': 0.10000007938858846, 'traj_error': -0.10000007938858846, 'motor': {'in_sync_mode': False, 'is_mg_accelerating': False, 'calibration_rcvd': True, 'pos': 14.579258918762207, 'near_vel_setpoint': False, 'pos_calibrated': True, 'at_current_limit': False, 'current': 0.05126507949829102, 'in_guarded_event': False, 'transport': {'transaction_time_max': 0, 'timestamp_pc': 0, 'read_error': 0, 'transaction_time_avg': 0, 'rate': 9.962835663131392, 'write_error': 0, 'itr': 174}, 'vel': -0.014918167144060135, 'pos_traj': 0.0, 'timestamp': 8722.013023, 'runstop_on': False, 'guarded_event': 10, 'trajectory_active': False, 'is_mg_moving': False, 'in_safety_event': False, 'effort': 4.272089958190918, 'is_moving': True, 'err': -0.0008841694798320532, 'timestamp_line_sync': 0.000000, 'near_pos_setpoint': True, 'mode': 5, 'diag': 277, 'debug': 0.0, 'waiting_on_sync': False}, 'traj_err': 0.0, 'vel': -0.0001023246728143643}
        # {'right_wheel': {'in_sync_mode': False, 'is_mg_accelerating': False, 'calibration_rcvd': True, 'pos': 3080.984619140625, 'near_vel_setpoint': False, 'pos_calibrated': False, 'at_current_limit': False, 'current': -6.726232628759122e-46, 'in_guarded_event': False, 'transport': {'transaction_time_max': 0, 'timestamp_pc': 0, 'read_error': 0, 'transaction_time_avg': 0, 'rate': 9.696512375219045, 'write_error': 0, 'itr': 83}, 'vel': -0.008616633713245392, 'pos_traj': 0.0, 'timestamp': 8726.154023, 'runstop_on': False, 'guarded_event': 0, 'trajectory_active': False, 'is_mg_moving': False, 'in_safety_event': False, 'effort': -5.605193857299268e-44, 'is_moving': True, 'err': 0.0, 'timestamp_line_sync': 0.000000, 'near_pos_setpoint': False, 'mode': 0, 'diag': 272, 'debug': 0.0, 'waiting_on_sync': False}, 'left_wheel': {'in_sync_mode': False, 'is_mg_accelerating': False, 'calibration_rcvd': True, 'pos': 3717.806396484375, 'near_vel_setpoint': False, 'pos_calibrated': False, 'at_current_limit': False, 'current': -6.726232628759122e-46, 'in_guarded_event': False, 'transport': {'transaction_time_max': 0, 'timestamp_pc': 0, 'read_error': 0, 'transaction_time_avg': 0, 'rate': 9.664829391487087, 'write_error': 0, 'itr': 83}, 'vel': 0.0058041587471961975, 'pos_traj': 0.0, 'timestamp': 8716.881023, 'runstop_on': False, 'guarded_event': 0, 'trajectory_active': False, 'is_mg_moving': False, 'in_safety_event': False, 'effort': -5.605193857299268e-44, 'is_moving': False, 'err': 0.0, 'timestamp_line_sync': 0.000000, 'near_pos_setpoint': False, 'mode': 0, 'diag': 256, 'debug': 0.0, 'waiting_on_sync': False}, 'y_vel': 0.0, 'x': 5.471624490368337e-06, 'effort': [0, 0], 'theta_vel': -0.0014532543963204673, 'pose_time_s': 8.030499999999847, 'rotation_torque': 0.0, 'timestamp_pc': 1664430119.952302, 'y': 1.3715419078241808e-10, 'theta': 6.283173738044901, 'translation_force': -2.849232141542364e-44, 'x_vel': -5.287051284726114e-05}
        # {'force': -1.5933265686035203, 'timestamp_pc': 1664430119.941239, 'pos': 0.5930173868111525, 'traj_error': -0.5930173868111525, 'motor': {'in_sync_mode': False, 'is_mg_accelerating': False, 'calibration_rcvd': True, 'pos': 62.10063552856445, 'near_vel_setpoint': False, 'pos_calibrated': True, 'at_current_limit': False, 'current': 0.5187556457519531, 'in_guarded_event': False, 'transport': {'transaction_time_max': 0, 'timestamp_pc': 0, 'read_error': 0, 'transaction_time_avg': 0, 'rate': 10.023261649345095, 'write_error': 0, 'itr': 489}, 'vel': -0.0321974940598011, 'pos_traj': 0.0, 'timestamp': 8730.281023, 'runstop_on': False, 'guarded_event': 15, 'trajectory_active': False, 'is_mg_moving': False, 'in_safety_event': False, 'effort': 43.229637145996094, 'is_moving': False, 'err': -0.00017470336752012372, 'timestamp_line_sync': 0.000000, 'near_pos_setpoint': True, 'mode': 8, 'diag': 261, 'debug': 0.0, 'waiting_on_sync': False}, 'vel': -0.00030746342008735694}
        # {'head_tilt': {'comm_errors': 0, 'vel_ticks': 0, 'overheating_error': False, 'temp': 41.0, 'timestamp_pc': 1664430119.895175, 'stalled': True, 'electrical_shock_error': False, 'pos': -0.01227184630308513, 'trajectory_active': False, 'stall_overload': False, 'overload_error': False, 'pos_ticks': 2056, 'hardware_error': 0, 'shutdown': 0, 'effort_ticks': -45, 'vel': -0.0, 'effort': -4.39453125, 'motor_encoder_error': False, 'input_voltage_error': False}, 'head_pan': {'comm_errors': 0, 'vel_ticks': 0, 'overheating_error': False, 'temp': 36.0, 'timestamp_pc': 1664430119.895175, 'stalled': True, 'electrical_shock_error': False, 'pos': -0.0015339807878856412, 'trajectory_active': False, 'stall_overload': False, 'overload_error': False, 'pos_ticks': 1166, 'hardware_error': 0, 'shutdown': 0, 'effort_ticks': -5, 'vel': -0.0, 'effort': -0.48828125, 'motor_encoder_error': False, 'input_voltage_error': False}}


    def publish_float64_msg(self, value, pub):
        msg = Float64()
        msg.data = value
        pub.publish(msg)

    def publish_bool_msg(self, value, pub):
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def publish_string_msg(self, value, pub):
        msg = String()
        msg.data = value
        pub.publish(msg)        


if __name__ == "__main__":

    node = HelloFormant()

    rate = rospy.Rate(10) # 10hz
    while True:
        # time.sleep(0.1)
        node.get_joints()
        # node.publish_odom_broadcast()
        rate.sleep()
        node.publish_pimu_status()
        node.publish_body_statuses()
        if not node.disable_amcl:
            node.publish_odom_amcl_broadcast()
