#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class GZVIOInterface():

    def __init__(self):
        rospy.init_node("gazebo_vio_publisher")

        
        rospy.loginfo("[GZVIOInterface] Starting GZVIOInterface as gazebo_vio_publisher.")

        rospy.Subscriber('/gazebo/iris/odom_gt', Odometry, self.gt_odom_cb, queue_size=1)
        self.vio_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, latch=False, queue_size=1)

    def gt_odom_cb(self, msg):
        # msg = Odometry()
        
        vio_pose = PoseStamped()
        vio_pose.header.frame_id = msg.header.frame_id
        vio_pose.header.stamp = rospy.Time.now()
        vio_pose.pose.position = msg.pose.pose.position
        vio_pose.pose.orientation = msg.pose.pose.orientation

        self.vio_pub.publish(vio_pose)


        


if __name__ == "__main__":
    voxl_tf_publisher = GZVIOInterface()
    rospy.spin()