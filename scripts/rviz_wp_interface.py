from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped, Quaternion, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import rospy
from copy import deepcopy

class RVizWPInterface():
    def __init__(self, height=2.0):
        rospy.loginfo("[RVizWPInterface] Waypoint Generator Node Started!")

        self.waypoint_marker_pub = rospy.Publisher('/wp_markers', MarkerArray, queue_size=10)
        
        # Subscribe to the Simple Goal topic
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        
        # Adjusted to correct message type for initialpose
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_pose_cb)

        self.marker_msg = MarkerArray()
        self.wp_list = []
        self.max_wp_count = rospy.get_param('/max_waypoint_count', 10)
        # self.wp_hgt = rospy.get_param('/waypoint_height', 2.0)
        self.wp_hgt = height
        self.ctr = 0
        self.flag_term_recvd = False

    def goal_cb(self, msg):
        if self.flag_term_recvd:
            rospy.loginfo("[RVizWPInterface] Initial pose set, not accepting more waypoints.")
            return

        point = PointStamped()
        point.header = msg.header
        point.point = msg.pose.position
        point.point.z = self.wp_hgt
        
        orientation = msg.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        
        self.process_new_waypoint(point, yaw)

    def initial_pose_cb(self, msg):
        rospy.loginfo("[RVizWPInterface] Initial pose received, finishing waypoints entry.")
        self.flag_term_recvd = True

    def process_new_waypoint(self, point, yaw):
        if self.ctr < self.max_wp_count and not self.flag_term_recvd:
            self.wp_list.append([point.point.x, point.point.y, point.point.z, yaw])
            self.ctr += 1
            rospy.loginfo(f'[RVizWPInterface] Waypoint {self.ctr} added')

            m = Marker()
            m.id = self.ctr
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.color = ColorRGBA(0, 1, 0, 1)
            m.scale = Vector3(0.5, 0.05, 0.05)  # Length and arrow shaft diameter
            m.header = point.header
            
            # Setting the orientation of the marker using quaternion
            quat = quaternion_from_euler(0, 0, yaw)
            m.pose.orientation.x = quat[0]
            m.pose.orientation.y = quat[1]
            m.pose.orientation.z = quat[2]
            m.pose.orientation.w = quat[3]
            m.pose.position = point.point
            
            self.marker_msg.markers.append(deepcopy(m))
            self.waypoint_marker_pub.publish(self.marker_msg)
        else:
            rospy.loginfo('[RVizWPInterface] Maximum waypoint count reached or initial pose set!')

    def reset_waypoints(self):
        self.ctr = 0
        self.wp_list = []
        self.marker_msg.markers.clear()
        rospy.loginfo('[RVizWPInterface] Waypoints cleared.')

# if __name__ == '__main__':
#     try:
#         rospy.init_node('waypoint_generator', anonymous=True)
#         nh = RVizWPInterface()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
