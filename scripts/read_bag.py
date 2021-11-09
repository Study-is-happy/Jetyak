import rospy
import rosbag
from br24.msg import BR24Scanline
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import threading
import copy
import scipy
import my_process
import my_util

gps_topic = '/vectornav/GPS'
scanline_topic = '/br24/scanline'
imu_topic = '/vectornav/IMU'

with rosbag.Bag('/home/zhiyongzhang/datasets/jetyak/jetyak_2019-03-27-11-20-10_42.bag') as bag:

    for topic, msg, t in bag.read_messages(
            topics=[gps_topic, scanline_topic, imu_topic]):

        if topic == gps_topic:
            print('gps-------------------')
        elif topic == scanline_topic:
            print('scanline')
        elif topic == imu_topic:
            print('imu')
