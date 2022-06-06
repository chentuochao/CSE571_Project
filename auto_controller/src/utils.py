# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import tf
import tf.transformations
from geometry_msgs.msg import Point32, Pose, Quaternion
from nav_msgs.srv import GetMap
from std_msgs.msg import Header
import numpy as np
import rospy

def angle_to_rosquaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


def rosquaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    _, _, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw


def rospose_to_posetup(posemsg):
    x = posemsg.position.x
    y = posemsg.position.y
    th = rosquaternion_to_angle(posemsg.orientation)
    return x, y, th


def angle_to_quaternion(angle):
    """
    Convert yaw angle in radians into a quaternion message
      angle: The yaw angle
      Returns: An equivalent geometry_msgs/Quaternion message
    """
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


def quaternion_to_angle(q):
    """
    Convert a quaternion message into a yaw angle in radians.
      q: A geometry_msgs/Quaternion message
      Returns: The equivalent yaw angle
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw


def rotation_matrix(theta):
    """
    Constructs a rotation matrix from a given angle in radians
      theta: The angle in radians
      Returns: The equivalent 2x2 numpy rotation matrix
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.matrix([[c, -s], [s, c]])


def particle_to_pose(particle):
    """
    Converts a particle to a pose message
      particle: The particle to convert - [x,y,theta]
      Returns: An equivalent geometry_msgs/Pose
    """
    pose = Pose()
    pose.position.x = particle[0]
    pose.position.y = particle[1]
    pose.orientation = angle_to_quaternion(particle[2])
    return pose


def particles_to_poses(particles):
    """
    Converts a list of particles to a list of pose messages
      particles: A list of particles, where each element is itself a list of the form [x,y,theta]
      Returns: A list of equivalent geometry_msgs/Pose messages
    """
    new_particle = []
    for i in range(0, particles.shape[0]):
        temp_result = particle_to_pose(particles[i, :])
        new_particle.append(temp_result)
    return new_particle  
    #return map(particle_to_pose, particles)


def make_header(frame_id, stamp=None):
    """
    Creates a header with the given frame_id and stamp. Default value of stamp is
    None, which results in a stamp denoting the time at which this function was called
      frame_id: The desired coordinate frame
      stamp: The desired stamp
      Returns: The resulting header
    """
    if stamp is None:
        stamp = rospy.Time.now()
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header


def map_to_world(poses, map_info):
    """
    Convert an array of pixel locations in the map to poses in the world. Does computations
    in-place
      poses: Pixel poses in the map. Should be a nx3 numpy array
      map_info: Info about the map (returned by get_map)
    """
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)
    print(angle)
    # Rotation
    c, s = np.cos(angle), np.sin(angle)

    # Store the x coordinates since they will be overwritten
    temp = np.copy(poses[:, 0])
    poses[:, 0] = c * poses[:, 0] - s * poses[:, 1]
    poses[:, 1] = s * temp + c * poses[:, 1]

    # Scale
    poses[:, :2] *= float(scale)

    # Translate
    poses[:, 0] += map_info.origin.position.x
    poses[:, 1] += map_info.origin.position.y
    poses[:, 2] += angle


def convert_to_pltaxis(state):
    state2 = np.zeros_like(state)
    state2[0] = 4000 - state[1]
    state2[1] = state[0]
    return state2
    
    
def world_to_map(poses, map_info):
    """
    Convert array of poses in the world to pixel locations in the map image
      pose: The poses in the world to be converted. Should be a nx3 numpy array
      map_info: Info about the map (returned by get_map)
    """
    scale = map_info.resolution
    angle = -quaternion_to_angle(map_info.origin.orientation)

    # Translation
    poses[:, 0] -= map_info.origin.position.x
    poses[:, 1] -= map_info.origin.position.y

    # Scale
    poses[:, :2] *= 1.0 / float(scale)
    # Rotation
    c, s = np.cos(angle), np.sin(angle)

    # Store the x coordinates since they will be overwritten
    temp = np.copy(poses[:, 0])
    poses[:, 0] = c * poses[:, 0] - s * poses[:, 1]
    poses[:, 1] = s * temp + c * poses[:, 1]
    poses[:, 2] += angle
    
    
def world_to_map_single(poses, map_info):
    """
    Convert array of poses in the world to pixel locations in the map image
      pose: The poses in the world to be converted. Should be a nx3 numpy array
      map_info: Info about the map (returned by get_map)
    """
    scale = map_info.resolution
    
    new_pose = np.zeros_like(poses)
    
    # Translation
    new_pose[0] = poses[0] - map_info.origin.position.x
    new_pose[1] = poses[1] - map_info.origin.position.y

    # Scale
    new_pose[:2] *= 1.0 / float(scale)
    
    return new_pose
