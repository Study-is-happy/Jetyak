import rospy

from mavros_msgs.srv import WaypointPush, WaypointClear, WaypointPull
from mavros_msgs.msg import WaypointList, Waypoint

import my_util


def update_waypoints(path):

    waypointList = WaypointList()

    for utm in path:
        latlng = my_util.utm_to_latlng(utm)
        waypoint = Waypoint()
        waypoint.frame = 3
        waypoint.command = 16
        waypoint.is_current = False
        waypoint.autocontinue = True
        waypoint.param1 = 0
        waypoint.param2 = 0
        waypoint.param3 = 0
        waypoint.param4 = 0
        waypoint.x_lat = latlng[0]
        waypoint.y_long = latlng[1]
        waypoint.z_alt = 100
        waypointList.waypoints.append(waypoint)

    waypointList.waypoints[0].is_current = True

    waypointPush_service = rospy.ServiceProxy(
        '/mavros/mission/push', WaypointPush)
    waypointPush_service.call(0, waypointList.waypoints)


def clear_waypoints():
    waypointClear_service = rospy.ServiceProxy(
        '/mavros/mission/clear', WaypointClear)
    waypointClear_service.call()
