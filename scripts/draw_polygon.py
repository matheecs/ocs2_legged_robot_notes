#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PolygonStamped, Point32
from jsk_recognition_msgs.msg import PolygonArray
from std_msgs.msg import Header
import numpy as np


def RectanglePolygon(index, header):
    p = PolygonStamped()
    p.header = header
    size = np.array([0.2, 1.0])
    center = np.array([0.3 * index, 0])
    vertex = np.array(
        [
            [center[0] + size[0] / 2, center[1] + size[1] / 2],
            [center[0] + size[0] / 2, center[1] - size[1] / 2],
            [center[0] - size[0] / 2, center[1] - size[1] / 2],
            [center[0] - size[0] / 2, center[1] + size[1] / 2],
        ]
    )
    z = -0.21
    p.polygon.points = [
        Point32(vertex[0][0], vertex[0][1], z),
        Point32(vertex[1][0], vertex[1][1], z),
        Point32(vertex[2][0], vertex[2][1], z),
        Point32(vertex[3][0], vertex[3][1], z),
    ]
    return p


if __name__ == "__main__":
    rospy.init_node("draw_polygon")
    pub = rospy.Publisher("polygon", PolygonArray, queue_size=10)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = PolygonArray()
        header = Header()
        header.frame_id = "odom"
        header.stamp = rospy.Time.now()
        msg.header = header

        for i in range(-50, 50):
            msg.polygons.append(RectanglePolygon(i, header))
            msg.labels.append(i + 50)
            msg.likelihood.append(np.random.ranf())

        pub.publish(msg)
        r.sleep()
