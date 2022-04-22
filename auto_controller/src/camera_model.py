import Queue
from threading import Lock

import numpy as np
import rospy
from sensor_msgs.msg import Image

class CameraModel:
    def __init__(
        self,
        rgb_topic,
        depth_topic,
        car_length,
        state_lock=None,
    ):
        if state_lock is None:
            self.state_lock = Lock()
        else:
            self.state_lock = state_lock

        # Subscribe to RGB image
        self.rgb_sub = rospy.Subscriber(
            rgb_topic, Image, self.rgb_cb, queue_size=1
        )

        # Subscribe to depth image
        self.depth_sub = rospy.Subscriber(
            depth_topic, Image, self.depth_cb, queue_size=1
        )


    def rgb_cb(self, msg):
        # Callback function fotr rgb image
        self.state_lock.acquire()
        rospy.loginfo("RGB is saving!")
        self.state_lock.release()

    def depth_cb(self, msg):
        # Callback function fotr Depth image
        self.state_lock.acquire()
        rospy.loginfo("Depth is saving!")
        self.state_lock.release()
