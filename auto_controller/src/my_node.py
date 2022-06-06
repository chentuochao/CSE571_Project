#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.
import time
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
from sensor_model import SensorModel
import utils
from Planner import My_planner
import os
import copy

from nav_msgs.msg import Path

#from camera_model import CameraModel 

# Publisher: /car/mux/ackermann_cmd_mux/input/navigation
# Subscriber: /scan /push_button_state /map /vesc/semsor/core /vesc/sensor/servo_position_command

class MyNode:
    def __init__(self,  
        motion_params,
        scan_topic,
        pos_topic,
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
        self.end_threshold = 1
        self.motion_params = motion_params
        self.control_rate = 5
        self._inferred_pose = None#np.array([0, 0, 0])#None
        MAP_TOPIC = "/map"


        mymap = np.load('/home/robot/catkin_ws/src/auto_controller/path/allen1_safeguard_notcrop_d1.npy')
        plan =np.load('/home/robot/catkin_ws/src/auto_controller/path/plan.npy')
        plan = plan[..., 0] 
        
        
        plan2 = np.zeros_like(plan)
        plan2[:, 0] = plan[:, 1]
        plan2[:, 1] = 4000 - plan[:, 0]
        
        #for i in range(0, plan2.shape[0]):
        #    point = utils.convert_to_pltaxis(plan2[i, :2])
        #    print(i, mymap[int(point[0])][int(point[1])]) 
        #raise KeyboardInterrupt
        
        traj = self.downsample_path(plan2)
                    
        self.landmark_pub = rospy.Publisher("~landmarks", PoseArray, queue_size=1)
        self.traj_pub = rospy.Publisher("~selected_traj", Path, queue_size=1)
        self.pose_pub = rospy.Publisher("~debug_pose", PoseStamped, queue_size=1)

        map_msg = rospy.wait_for_message(MAP_TOPIC, OccupancyGrid)
        self.map_info = map_msg.info
        self.resolution = map_msg.info.resolution
        print(map_msg.info)
        
        utils.map_to_world(traj, self.map_info)
        
        #temp = copy.deepcopy(traj)
        #utils.world_to_map(temp, self.map_info)
        
        self.traj = traj
        self.publish_landmarks(self.traj)

        self.planner = My_planner( mymap, self.map_info, motion_params, traj, 1.0/self.control_rate)

        self.inferred_pose_lock = threading.Lock()

        self.rp_ctrls = rospy.Publisher(
            car_name + ctrl_topic,
            AckermannDriveStamped,
            queue_size=2,
        )
        
        self.sensor_model = SensorModel(
            scan_topic,
            laser_ray_step,
            exclude_max_range_rays,
            max_range_meters,
            map_msg,
            car_length
        )
        
        #print(pos_topic) 
        rospy.Subscriber(
            pos_topic,
            PoseStamped,
            self.cb_pose,
            queue_size=10,
        )

        self.logger.info("My node: Initialized")

    def cb_pose(self, msg):
        #self.logger.info("recv ip ok")
        ip = utils.rospose_to_posetup(msg.pose)
        ip = np.array(ip) 
        with self.inferred_pose_lock:
            self._inferred_pose = ip
    
    def get_inferred_pose(self):
        with self.inferred_pose_lock:
            return self._inferred_pose
  
    def downsample_path(self, plan):
        step = 30
        half_win = int(step/2)
        desample_plan = plan[half_win:plan.shape[0]:step, :]
        N_downsample = len(desample_plan) 
        
        desample_traj = np.zeros((N_downsample + 1, 3))
        start = half_win
        for i in range(0, N_downsample): 
            thetas = []
            segment = plan[start - half_win :start+half_win+1]  
            theta =np.arctan2((segment[-1, 1]-segment[0, 1]),(segment[-1, 0]-segment[0, 0]))
            desample_traj[i, 0:2] = desample_plan[i]
            desample_traj[i, 2] = theta
            start += step
            
        desample_traj[N_downsample,:2] = np.array(plan[-1, :]) 
        desample_traj[N_downsample,2] = np.pi/2.0
        
        return desample_traj
        
    def start(self):
        self.logger.info("Starting MyController")
    
        rate = rospy.Rate(50)
        last_update = time.time()
        ctrl = [0, 0]
        selected_traj = None
        momentum = 0

        while not rospy.is_shutdown():
            self.publish_landmarks(self.traj)
            if time.time() - last_update >= 1.0/self.control_rate:
                last_update = time.time()
                ip = self.get_inferred_pose()
                ob = self.sensor_model.get_lidar_ob()
                if ip is not None and ob is not None:       
                    #print(ob)
                    ctrl_temp, selected_traj = self.planner.motion_planning(ip, ob)
                    if ctrl_temp is not None:
                      self.publish_debug_pose(ip, ctrl_temp[1])
                    if selected_traj is not None:
                      self.publish_traj(selected_traj)
  
                      ctrl[0] = ctrl_temp[0]
                      ctrl[1] = (1-momentum)*ctrl_temp[1] + momentum*ctrl[1]
                    else:
                      ctrl = [0, 0]
                      #ctrl[0] = 0

            #if selected_traj is not None:
            #    self.publish_traj(selected_traj)  

            if ctrl is not None:
                self.publish_control(ctrl)
            else:
                self.publish_control([0, 0])

            rate.sleep()
    
    def publish_debug_pose(self, ip, delta):
        #print("Publish debug_pos: ", delta)
        ps = PoseStamped()
        ps.header = utils.make_header("map")
        ps.pose.position.x = ip[0]
        ps.pose.position.y = ip[1]
        ps.pose.orientation = utils.angle_to_quaternion(delta)
        self.pose_pub.publish(ps)


    def publish_traj(self, traj):
        msg = Path()
        msg.header = utils.make_header("map")
        print("Publish traj:", traj[0, :2])
        for t in range(0, traj.shape[0]):
            pose = PoseStamped()
            pose.pose.position.x = traj[t, 0]
            pose.pose.position.y = traj[t, 1]
            pose.pose.position.z = 0
            msg.poses.append(pose)
        self.traj_pub.publish(msg)
        
    def publish_landmarks(self, particles):
        """
        Helper function for publishing a pose array of particles
          particles: To particles to publish
        """
        #print(particles)
        pa = PoseArray()
        pa.header = utils.make_header("map")
        pa.poses = utils.particles_to_poses(particles)
        self.landmark_pub.publish(pa)

    def publish_control(self, ctrl):
        ctrlmsg = AckermannDriveStamped()
        ctrlmsg.header.stamp = rospy.Time.now()
        ctrlmsg.drive.speed = ctrl[0]
        ctrlmsg.drive.steering_angle = ctrl[1]
        self.rp_ctrls.publish(ctrlmsg)
        #self.logger.info("My node: sending")
        


if __name__ == "__main__":
    rospy.init_node("My_node", anonymous=True)  # Initialize the node

    # Car name

    car_name = rospy.get_param("~car_name")
    # The topic containing servo state information

    ctrl_topic = rospy.get_param(
        "~ctrl_topic", "/mux/ackermann_cmd_mux/input/navigation"
    )

    ### params for the laser sensor
    scan_topic = rospy.get_param("~scan_topic", "/scan")
    pos_topic = rospy.get_param("~car_pos", "/particle_filter/inferred_pose")

    # Step for downsampling laser scans
    laser_ray_step = int(rospy.get_param("~laser_ray_step"))
    exclude_max_range_rays = bool(rospy.get_param("~exclude_max_range_rays"))
    # The max range of the laser
    max_range_meters = float(rospy.get_param("~max_range_meters"))

    ## params for the motion model
    # Offset conversion param from rpm to speed
    speed_to_erpm_offset = float(rospy.get_param(car_name + "/vesc/speed_to_erpm_offset", 0.0))
    # Gain conversion param from rpm to speed
    speed_to_erpm_gain = float(rospy.get_param(car_name + "/vesc/speed_to_erpm_gain", -3750))
    # Offset conversion param from servo position to steering angle
    steering_angle_to_servo_offset = float(
        rospy.get_param(car_name + "/vesc/steering_angle_to_servo_offset",0.518)
    )
    # Gain conversion param from servo position to steering angle
    steering_angle_to_servo_gain = float(
        rospy.get_param(car_name + "/vesc/steering_angle_to_servo_gain", 0.84)
    )
    # The length of the car
    car_length = float(rospy.get_param("/car_kinematics/car_length", 0.33))

    motion_params = {}
    motion_params["L"] = car_length
    motion_params["speed_to_erpm_offset"] = speed_to_erpm_offset
    motion_params["speed_to_erpm_gain"] = speed_to_erpm_gain
    motion_params["steering_angle_to_servo_offset"] = steering_angle_to_servo_offset
    motion_params["steering_angle_to_servo_gain"] = steering_angle_to_servo_gain
    motion_params["max_delta"] = 0.34

    node0 =  MyNode(
        motion_params,
        car_name + scan_topic,
        car_name + pos_topic,
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
    time.sleep(50)
    node0.start()
