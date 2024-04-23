#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from pose_transformer import MavTfTransformer
from voxl_msgs.msg import TagDetection
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from copy import deepcopy


class FiducialTagInterface():

    def __init__(self):
        # rospy.init_node("fiducial_tag_interface")

        self.tag_pose = None
        self.flag_data_timeout = False
        self.latest_pose_stamp = rospy.Time.now()
        self.flag_data_recvd = False
        self.fixed_frame_id = "map"

        self.data_timeout_sec = rospy.get_param('~tag_pose_timeout', 2.0)

        # 0 - VOXL AprilTag
        # 1 - Aruco Tag
        self.fiducial_type = rospy.get_param('~fiducial_tag_type',0) 
        self.aruco_tag_id = rospy.get_param('~target_aruco_tag_id', int(1))

        # used for frame transformations
        self.tf_obj = MavTfTransformer()
        rospy.loginfo("[FiducialTagInterface] Fiducial tag interface loaded!")

        # When using Aruco Detect ROS pkg
        if self.fiducial_type==1:
            rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.aruco_pose_cb, queue_size=1)
        # When using tag detection interface from VOXL-MPA-ROS
        elif self.fiducial_type==0:
            rospy.Subscriber('/tag_detections', TagDetection, self.voxl_tag_pose_cb, queue_size=1)

        
        self.data_checker_loop = rospy.Timer(rospy.Duration(0.10), self.data_check_cb)
        # For visualisation
        self.lt_pub = rospy.Publisher('/fiducial_interface/tag_pose_ff', PoseStamped, latch=False, queue_size=1)
        self.lt_vis_pub = rospy.Publisher('/fiducial_interface/tag_vis_marker', MarkerArray, latch=False, queue_size=1)

        
    # Process ARUCO detections
    def aruco_pose_cb(self, msg):

        if not self.flag_data_recvd and len(msg.transforms)>0:
            rospy.loginfo("[FiducialTagInterface] Tag pose data recovered!")
            self.flag_data_recvd = True
        
        if len(msg.transforms)>0:
            self.latest_pose_stamp = rospy.Time.now()

            for m in msg.transforms:
                # use only the pose of the correct tag ID
                if m.fiducial_id == self.aruco_tag_id:
                    p = PoseStamped()
                    p.header.frame_id = "depthcamera_link"
                    p.header.stamp = rospy.Time.now()
                    p.pose.position.x = m.transform.translation.x
                    p.pose.position.y = m.transform.translation.y
                    p.pose.position.z = m.transform.translation.z
                    p.pose.orientation.x = m.transform.rotation.x
                    p.pose.orientation.y = m.transform.rotation.y
                    p.pose.orientation.z = m.transform.rotation.z
                    p.pose.orientation.w = m.transform.rotation.w

                    res = self.tf_obj.transform_pose(p,to_frame=self.fixed_frame_id)
                    self.tag_pose = res[0]
                    self.lt_pub.publish(res[0])
                    self.publish_tag_marker(res[0])


    # Process AprilTag detections
    def voxl_tag_pose_cb(self, msg):
        if not self.flag_data_recvd:
            self.flag_data_recvd = True
        
        self.latest_pose_stamp = rospy.Time.now()
        # transform Tag pose from camera frame to fixed frame
        res = self.tf_obj.transform_pose(msg.pose,to_frame=self.fixed_frame_id)
        self.tag_pose = res[0]
        self.tag_pose.header.stamp = rospy.Time.now()
        
        self.lt_pub.publish(res[0])
        self.publish_tag_marker(res[0])
                

    def publish_tag_marker(self, pose):
        marker_arr_msg = MarkerArray()

        # Rectangle
        m = Marker()
        m.id = 0
        m.action = m.ADD
        m.header = pose.header
        m.color = ColorRGBA(0,1,0,1)
        m.type = m.CUBE
        m.scale.x = 0.50
        m.scale.y = 0.50
        m.scale.z = 0.02
        m.pose = pose.pose
        marker_arr_msg.markers.append(deepcopy(m))


        # Text
        m = Marker()
        m.id = 1
        m.action = m.ADD
        m.header = pose.header
        m.color = ColorRGBA(1,1,1,1)
        m.type = m.TEXT_VIEW_FACING
        m.scale.z = 0.25
        m.pose = pose.pose
        m.pose.position.z += 0.25
        m.text = "Landing Tag"
        marker_arr_msg.markers.append(deepcopy(m))

        self.lt_vis_pub.publish(marker_arr_msg)

    
    def get_tag_pose(self):
        return self.tag_pose

    def set_fixed_frame(self, frame_id):
        self.fixed_frame_id = frame_id
    
    # timer callback to check for detection timeout
    def data_check_cb(self, event):

        if not self.flag_data_timeout and self.flag_data_recvd:
            if self.flag_data_recvd and (rospy.Time.now()-self.latest_pose_stamp)>rospy.Duration(self.data_timeout_sec):
                rospy.logerr("[FiducialTagInterface] Tag pose data timed-out!")
                self.flag_data_recvd = False
                # self.tag_pose = None