import Queue
from threading import Lock

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

THETA_DISCRETIZATION = 112  # Discretization of scanning angle
INV_SQUASH_FACTOR = 0.2  # Factor for helping the weight distribution to be less peaked

Z_SHORT = 0.1  # Weight for short reading
Z_MAX = 0.05  # Weight for max reading
Z_RAND = 0.05  # Weight for random reading
SIGMA_HIT = 8.0  # Noise value for hit reading
Z_HIT = 0.80  # Weight for hit reading

"""
  Weights particles according to their agreement with the observed data
"""


class SensorModel:
    def __init__(
        self,
        scan_topic,
        laser_ray_step,
        exclude_max_range_rays,
        max_range_meters,
        map_msg,
        car_length,
        state_lock=None,
    ):

        """
        Initializes the sensor model
          scan_topic: The topic containing laser scans
          laser_ray_step: Step for downsampling laser scans
          exclude_max_range_rays: Whether to exclude rays that are beyond the max range
          max_range_meters: The max range of the laser
          map_msg: A nav_msgs/MapMetaData msg containing the map to use
          state_lock: Used to control access to particles and weights
        """
        if state_lock is None:
            self.state_lock = Lock()
        else:
            self.state_lock = state_lock

        self.LASER_RAY_STEP = laser_ray_step  # Step for downsampling laser scans
        self.EXCLUDE_MAX_RANGE_RAYS = (
            exclude_max_range_rays  # Whether to exclude rays beyond the max range
        )
        self.MAX_RANGE_METERS = max_range_meters  # The max range of the laser
        self.CAR_LENGTH = car_length

        # The max range in pixels of the laser
        #self.max_range_px = int(self.MAX_RANGE_METERS / map_msg.info.resolution)

        # Load the sensor model expressed as a table
        self.queries = None
        self.ranges = None
        self.laser_angles = None  # The angles of each ray
        #self.laser_data = None
        self.downsampled_angles = None  # The angles of the downsampled rays
        self.downsampled_ranges = None
        # Set so that outside code can know that it's time to resample
        self.do_resample = False

        # Subscribe to laser scans
        self.laser_sub = rospy.Subscriber(
            scan_topic, LaserScan, self.lidar_cb, queue_size=1
        )
        
    def get_lidar_ob(self):
        if self.downsampled_angles is None or self.downsampled_ranges is None:
            return None
            
        self.state_lock.acquire()
        obs = (
                np.copy(self.downsampled_ranges).astype(np.float32),
                np.copy(self.downsampled_angles).astype(np.float32),
            )
        self.state_lock.release()
        return obs

    def lidar_cb(self, msg):
        """
        Downsamples laser measurements and applies sensor model
          msg: A sensor_msgs/LaserScan
        """
        
        self.state_lock.acquire()
        #rospy.loginfo("Lidar is saving!")
        # Down sample the laser rays
        if not self.EXCLUDE_MAX_RANGE_RAYS:
            # Initialize angle arrays
            if not isinstance(self.laser_angles, np.ndarray):
                self.laser_angles = np.linspace(
                    msg.angle_min, msg.angle_max, len(msg.ranges)
                )
                self.downsampled_angles = np.copy(
                    self.laser_angles[0 :: self.LASER_RAY_STEP]
                ).astype(np.float32)

            self.downsampled_ranges = np.array(
                msg.ranges[:: self.LASER_RAY_STEP]
            )  # Down sample
            self.downsampled_ranges[
                np.isnan(self.downsampled_ranges)
            ] = self.MAX_RANGE_METERS  # Remove nans
            self.downsampled_ranges[
                self.downsampled_ranges[:] == 0
            ] = self.MAX_RANGE_METERS  # Remove 0 values

        else:
            # Initialize angle array
            if not isinstance(self.laser_angles, np.ndarray):
                self.laser_angles = np.linspace(
                    msg.angle_min, msg.angle_max, len(msg.ranges)
                )
            ranges = np.array(msg.ranges)  # Get the measurements
            ranges[np.isnan(ranges)] = self.MAX_RANGE_METERS  # Remove nans
            # Find non-extreme measurements
            valid_indices = np.logical_and(
                ranges > 0.01, ranges < self.MAX_RANGE_METERS
            )
            # Get angles corresponding to non-extreme measurements
            self.filtered_angles = np.copy(self.laser_angles[valid_indices]).astype(
                np.float32
            )
            # Get non-extreme measurements
            self.filtered_ranges = np.copy(ranges[valid_indices]).astype(np.float32)

            # Compute expected number of rays
            ray_count = int(self.laser_angles.shape[0] / self.LASER_RAY_STEP)
            # Get downsample indices
            sample_indices = np.arange(
                0,
                self.filtered_angles.shape[0],
                float(self.filtered_angles.shape[0]) / ray_count,
            ).astype(np.int)

            # Initialize down sample angles
            self.downsampled_angles = np.zeros(ray_count + 1, dtype=np.float32)
            # Initialize down sample measurements
            self.downsampled_ranges = np.zeros(ray_count + 1, dtype=np.float32)

            # Populate downsample angles
            self.downsampled_angles[: sample_indices.shape[0]] = np.copy(
                self.filtered_angles[sample_indices]
            )
            # Populate downsample measurements
            self.downsampled_ranges[: sample_indices.shape[0]] = np.copy(
                self.filtered_ranges[sample_indices]
            )

        # Compute the observation
        # obs is a a two element tuple
        # obs[0] is the downsampled ranges
        # obs[1] is the downsampled angles
        # Each element of obs must be a numpy array of type np.float32
        # Use self.LASER_RAY_STEP as the downsampling step
        # Keep efficiency in mind, including by caching certain things that won't change across future iterations of this callback
    
        ## -------------- process the laser data -----------------
        self.state_lock.release()
