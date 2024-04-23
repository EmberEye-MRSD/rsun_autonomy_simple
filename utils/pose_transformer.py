#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs


class MavTfTransformer():
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        self.success = False

    def transform_pose(self, pose_input, to_frame="map"):
        try:
            trans, success = self.get_transform( pose_input.header.frame_id, to_frame )
            output_pose = tf2_geometry_msgs.do_transform_pose(pose_input, trans)
            self.success = success
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.success = False
            raise
        return output_pose, self.success
    
    def get_transform(self, source_frame, target_frame):
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.2) )
            self.success = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.success = False
            # rospy.logerr(f'Cannot find transformation from {source_frame} to {target_frame}')
            # raise Exception(f'Cannot find transformation from {source_frame} to {target_frame}') from e
        return trans, self.success     # Type: TransformStamped