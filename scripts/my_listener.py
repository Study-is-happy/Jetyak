import rospy
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
import cache_satellite_images


jetyak_config = {'length': 3.3, 'width': 1.,
                 'radar_distance': 200., 'radar_dt': 2.5, 'radar_offset': np.radians(3),
                 'utm': None, 'yaw': None, 'speed': 0.}

process_thread = threading.Thread()
process_thread.start()
previous_angle = 0
radar_data = {'scanline': [], 'utm': [], 'angle': []}


def NavSatFixListener(data):

    jetyak_config['utm'] = my_util.latlng_to_utm((data.latitude, data.longitude))


def ImuListener(data):

    jetyak_config['yaw'] = scipy.spatial.transform.Rotation.from_quat([data.orientation.x, data.orientation.y,
                                                                       data.orientation.z, data.orientation.w]).as_euler('zyx')[0]


def OdomListener(data):

    linear = data.twist.twist.linear
    jetyak_config['speed'] = np.sqrt(linear.x**2 + linear.y**2)


def BR24ScanlineListener(data):

    if jetyak_config['utm'] is None or jetyak_config['yaw'] is None:
        return

    global process_thread
    global previous_angle
    global radar_data

    if data.angle < previous_angle:

        process_thread.join()

        process_thread = threading.Thread(target=my_process.process, args=(
            copy.deepcopy(radar_data), jetyak_config, data.header.stamp))

        # process_thread = threading.Thread(target=cache_satellite_images.cache_satellite_images, args=(
        #     radar_data['utm'][-1]))

        process_thread.start()

        radar_data = {'scanline': [], 'utm': [], 'angle': []}

    radar_data['scanline'].append(data.scanline_data)
    radar_data['utm'].append(jetyak_config['utm'])
    radar_data['angle'].append(
        jetyak_config['yaw'] + jetyak_config['radar_offset'] + np.pi * 2 * data.angle / 4096.)

    previous_angle = data.angle


def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/vectornav/GPS',
                     NavSatFix, NavSatFixListener)

    rospy.Subscriber('/br24/scanline', BR24Scanline, BR24ScanlineListener)

    rospy.Subscriber('/vectornav/IMU', Imu, ImuListener)

    rospy.Subscriber('/mavros/local_position/odom', Odometry, OdomListener)

    rospy.spin()


if __name__ == '__main__':
    listener()
