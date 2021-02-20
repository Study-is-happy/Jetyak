import rospy
from br24.msg import BR24Scanline
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import threading
import copy
import tf
import my_process
import my_util


config = {"self_length": 3.3, "self_width": 1.,
          "radar_distance": 200., "radar_dt": 2.5,
          "self_utm": None, "self_angle": None, "self_speed": 0.}

config["self_bevel"] = np.sqrt(
    config["self_length"]**2 + config["self_width"])

radar_data = {"scanline": [], "utm": [], "angle": []}

previous_angle = 0


def NavSatFixListener(data):

    config["self_utm"] = my_util.latlng_to_utm((data.latitude, data.longitude))


def ImuListener(data):

    quaternion = (data.orientation.x, data.orientation.y,
                  data.orientation.z, data.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    config["self_angle"] = euler[2]


def OdomListener(data):

    linear = data.twist.twist.linear
    config["self_speed"] = np.sqrt(linear.x**2 + linear.y**2)


def BR24ScanlineListener(data):

    if config["self_utm"] is None or config["self_angle"] is None:
        return

    global previous_angle
    global radar_data

    if data.angle < previous_angle:

        threading.Thread(target=my_process.process, args=(
            copy.deepcopy(radar_data), config)).start()

        radar_data = {"scanline": [], "utm": [], "angle": []}

    radar_data["scanline"].append(data.scanline_data)
    radar_data["utm"].append(config["self_utm"])
    radar_data["angle"].append(
        config["self_angle"] + np.pi * 2 * data.angle / 4096.)

    previous_angle = data.angle


def listener():

    rospy.init_node("listener", anonymous=True)

    rospy.Subscriber("/vectornav/GPS",
                     NavSatFix, NavSatFixListener)

    rospy.Subscriber("/br24/scanline", BR24Scanline, BR24ScanlineListener)

    rospy.Subscriber("/vectornav/IMU", Imu, ImuListener)

    rospy.Subscriber("/mavros/local_position/odom", Odometry, OdomListener)

    rospy.spin()


if __name__ == '__main__':
    listener()
