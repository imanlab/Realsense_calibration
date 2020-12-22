#!/usr/bin/env python

import math
import rospy
import numpy as np
# from collections import deque
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from whycon_ros.msg import MarkerArray


class OrientationGroundTruth(object):
    def __init__(self):
        super(OrientationGroundTruth, self).__init__()
        self.marker_seperation = 0.105
        self.marker_size = 0.079
        self.colours = dict()
        self.colours['red'] = (255, 0, 0)
        self.colours['green'] = (0, 255, 0)
        self.colours['blue'] = (0, 0, 255)
        self.colours['yellow'] = (255, 255, 0)
        self.colours['black'] = (0, 0, 0)
        self.colours['white'] = (255, 255, 255)
        self.sliding_window = np.zeros(12)
        print self.sliding_window
        self.sub = rospy.Subscriber('/whycon_ros/markers', MarkerArray, self.callback)
        self.pub_pose = rospy.Publisher('/our/test/pose_stamped', PoseStamped, queue_size=1)
        self.pub_marker = rospy.Publisher('/our/test/marker', Marker, queue_size=1)

    def callback(self, msg):
        if len(msg.markers) != 4 or (sorted([i.id for i in msg.markers]) != [1, 2, 16, 17] and sorted([i.id for i in msg.markers]) != [1, 8, 24, 30] and sorted([i.id for i in msg.markers]) != [14, 15, 29, 30] and sorted([i.id for i in msg.markers]) != [1, 8, 16, 24] and sorted([i.id for i in msg.markers]) != [1, 2, 16, 17]):
            print "Incorrect IDs detected, skipping frame...", sorted([i.id for i in msg.markers])
            return

        #print(msg.markers[0].id, msg.markers[1].id, msg.markers[2].id, msg.markers[3].id)
        origin, right, opposite, left = sorted(msg.markers, key=lambda x: x.id)

        live_readings = np.array([
            origin.position.position.x, origin.position.position.y, origin.position.position.z,
            right.position.position.x, right.position.position.y, right.position.position.z,
            opposite.position.position.x, opposite.position.position.y, opposite.position.position.z,
            left.position.position.x, left.position.position.y, left.position.position.z])

        bias = 64
        w = 2
        self.sliding_window = np.add(self.sliding_window * (bias - w), live_readings * w) / (bias * 2)

        #print self.sliding_window


        origin_x, origin_y, origin_z,\
            right_x, right_y, right_z,\
            opposite_x, opposite_y, opposite_z,\
            left_x, left_y, left_z = live_readings

        # =========================== Find vector X ===========================
        vx = np.array([right_x - origin_x,
                right_y - origin_y,
                right_z - origin_z]) / ((self.marker_seperation + self.marker_size) * 3)

        # =========================== Find vector Y ===========================
        vy = np.array([left_x - origin_x,
                left_y - origin_y,
                left_z - origin_z]) / ((self.marker_seperation + self.marker_size) * 3)

        # =========================== Find vector Z ===========================
        vz = np.cross(vx, vy)

        # ======================== Convert to ROS Msg =========================
        p_msg = self.pose_from_vector_3d(vz, msg.header)

        p_msg.pose.position.x = (origin_x + opposite_x) / 2
        p_msg.pose.position.y = (origin_y + opposite_y) / 2
        p_msg.pose.position.z = (origin_z + opposite_z) / 2

        print(p_msg.pose.position.z)
        self.pub_pose.publish(p_msg)

        for m in sorted(msg.markers, key=lambda x: x.id):
            marker = Marker()
            marker.header = msg.header
            marker.header.frame_id = '/camera_color_optical_frame'
            marker.header.stamp = rospy.Time.now()
            marker.id = m.id
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position = m.position.position
            marker.pose.orientation.x = m.position.orientation.x
            marker.pose.orientation.y = m.position.orientation.y
            marker.pose.orientation.z = m.position.orientation.z
            marker.pose.orientation.w = m.position.orientation.w
            marker.scale.x = 0.005
            marker.scale.y = 0.005
            marker.scale.z = 0.005
            marker.color.a = 1.0
            if m.id == 6:
                marker.color.r, marker.color.g, marker.color.b = self.colours['red']
            elif m.id == 7:
                marker.color.r, marker.color.g, marker.color.b = self.colours['green']
            elif m.id == 8:
                marker.color.r, marker.color.g, marker.color.b = self.colours['blue']
            elif m.id == 9:
                marker.color.r, marker.color.g, marker.color.b = self.colours['yellow']
            else:
                marker.color.r, marker.color.g, marker.color.b = self.colours['black']
                marker.color.r, marker.color.g, marker.color.b = self.colours['blue']
            self.pub_marker.publish(marker)

    def pose_from_vector_3d(self, normal_z, header):
        # print normal_z
        msg = PoseStamped()
        msg.header = header
        msg.header.frame_id = "/camera_color_optical_frame"
        msg.pose.position.x = normal_z[0]
        msg.pose.position.y = normal_z[1]
        msg.pose.position.z = normal_z[2]
        #  calculating the half-way vector.
        u = [0, 0, 1]
        norm = np.linalg.norm(normal_z)
        v = np.asarray(normal_z) / norm
        if (np.array_equal(u, v)):
            msg.pose.orientation.w = 1
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
        elif (np.array_equal(u, np.negative(v))):
            msg.pose.orientation.w = 0
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 1
        else:
            half = [u[0] + v[0], u[1] + v[1], u[2] + v[2]]
            msg.pose.orientation.w = np.dot(u, half)
            temp = np.cross(u, half)
            msg.pose.orientation.x = temp[0]
            msg.pose.orientation.y = temp[1]
            msg.pose.orientation.z = temp[2]
        norm = math.sqrt(msg.pose.orientation.x * msg.pose.orientation.x + msg.pose.orientation.y * msg.pose.orientation.y +
            msg.pose.orientation.z * msg.pose.orientation.z + msg.pose.orientation.w * msg.pose.orientation.w)
        if norm == 0:
            norm = 1
        msg.pose.orientation.x /= norm
        msg.pose.orientation.y /= norm
        msg.pose.orientation.z /= norm
        msg.pose.orientation.w /= norm
        return msg


def main():
    OrientationGroundTruth()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("orientation_ground_truth")
    main()
