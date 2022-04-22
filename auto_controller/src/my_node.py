#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import cProfile
import os
import signal
import threading
import Queue
from threading import Lock
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped,  PoseArray, PoseWithCovarianceStamped
from std_msgs.msg import ColorRGBA, Empty
from std_srvs.srv import Empty as SrvEmpty
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

import logger
from motion_model import KinematicMotionModel
from sensor_model import SensorModel
from camera_model import CameraModel 
# Publisher: /car/mux/ackermann_cmd_mux/input/navigation
# Subscriber: /scan /push_button_state /camera /map /vesc/semsor/core /vesc/sensor/servo_position_command

class MyNode:
    def __init__(self,  
        motor_state_topic,
        servo_state_topic,
        scan_topic,
        rgb_topic,
        depth_topic,
        laser_ray_step,
        exclude_max_range_rays,
        max_range_meters,
        speed_to_erpm_offset,
        speed_to_erpm_gain,
        steering_angle_to_servo_offset,
        steering_angle_to_servo_gain,
        car_length,
        car_name,
    ):
        self.name = car_name
        self.state_lock = Lock()
        self.logger = logger.RosLog()

        MAP_TOPIC = "/map"
        map_msg = None
        #map_msg = rospy.wait_for_message(MAP_TOPIC, OccupancyGrid)
        

        # control instruction publisher to send the control instruction to car
        self.rp_ctrls = rospy.Publisher(
            car_name + ctrl_topic,
            AckermannDriveStamped,
            queue_size=2,
        )


        # Subscriber for the laser sensors to get the stream of laser data
        self.sensor_model = SensorModel(
            scan_topic,
            laser_ray_step,
            exclude_max_range_rays,
            max_range_meters,
            map_msg,
            car_length,
            self.state_lock,
        )

        # Subscriber for the camera for the RGB image and depth image
        self.camera_model = CameraModel(
            rgb_topic,
            depth_topic,
            car_length,
            self.state_lock,
        )
        
        # Subscriber for the motion sensor to acquire the current speed and steer
        self.motion_model = KinematicMotionModel(
            motor_state_topic,
            servo_state_topic,
            speed_to_erpm_offset,
            speed_to_erpm_gain,
            steering_angle_to_servo_offset,
            steering_angle_to_servo_gain,
            car_length,
            [0, 0, 0],
            self.state_lock,
        )
        self.logger.info("My node: Initialized")

    def start(self):
        self.logger.info("Starting MyController")
    
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ctrl = [0, np.pi/4.0]
            self.publish_control(ctrl)
            rate.sleep()


    def publish_control(self, ctrl):
        ## to send the control instruction to the controller
        ctrlmsg = AckermannDriveStamped()
        ctrlmsg.header.stamp = rospy.Time.now()
        ctrlmsg.drive.speed = ctrl[0]
        ctrlmsg.drive.steering_angle = ctrl[1]
        self.rp_ctrls.publish(ctrlmsg)
        self.logger.info("My node: sending")
        


if __name__ == "__main__":
    rospy.init_node("My_node", anonymous=True)  # Initialize the node

    # Car name
    car_name = rospy.get_param("~car_name")
    motor_state_topic = rospy.get_param("~motor_state_topic", "/vesc/sensors/core")
    # The topic containing servo state information
    servo_state_topic = rospy.get_param(
        "~servo_state_topic", "/vesc/sensors/servo_position_command"
    )

    ctrl_topic = rospy.get_param(
        "~ctrl_topic", "/mux/ackermann_cmd_mux/input/navigation"
    )

    ### params for the laser sensor
    scan_topic = rospy.get_param("~scan_topic", "/scan")
    rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")

    depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_rect_raw")
    # Step for downsampling laser scans
    laser_ray_step = int(rospy.get_param("~laser_ray_step"))
    exclude_max_range_rays = bool(rospy.get_param("~exclude_max_range_rays"))
    # The max range of the laser
    max_range_meters = float(rospy.get_param("~max_range_meters"))

    ## params for the motion model
    # Offset conversion param from rpm to speed
    speed_to_erpm_offset = float(rospy.get_param(car_name + "/vesc/speed_to_erpm_offset", 0.0))
    # Gain conversion param from rpm to speed
    speed_to_erpm_gain = float(rospy.get_param(car_name + "/vesc/speed_to_erpm_gain", 4350))
    # Offset conversion param from servo position to steering angle
    steering_angle_to_servo_offset = float(
        rospy.get_param(car_name + "/vesc/steering_angle_to_servo_offset", 0.5)
    )
    # Gain conversion param from servo position to steering angle
    steering_angle_to_servo_gain = float(
        rospy.get_param(car_name + "/vesc/steering_angle_to_servo_gain", -1.2135)
    )
    # The length of the car
    car_length = float(rospy.get_param("/car_kinematics/car_length", 0.33))

    node0 =  MyNode(
        car_name + motor_state_topic,
        car_name +servo_state_topic,
        car_name +scan_topic,
        car_name + rgb_topic,
        car_name + depth_topic,
        laser_ray_step,
        exclude_max_range_rays,
        max_range_meters,
        speed_to_erpm_offset,
        speed_to_erpm_gain,
        steering_angle_to_servo_offset,
        steering_angle_to_servo_gain,
        car_length,
        car_name,
    )

    node0.start()
