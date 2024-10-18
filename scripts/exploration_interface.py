import rospy
from geometry_msgs.msg import PoseStamped,Quaternion
from nav_msgs.msg import Odometry, Path
from mavros_msgs.msg import PositionTarget
from quadrotor_msgs.msg import PositionCommand
from exploration_manager.msg import ExplorationState
from interface_state import InterfaceStates
from base_interface import BaseInterface


class ExplorationInterface(BaseInterface):

    def __init__(self):
        # Exploration Trigger
        rospy.loginfo("[ExplorationInterface] Loaded!")
        self.trigger_pub = rospy.Publisher('/waypoint_generator/waypoints', Path, queue_size=10)
        self.exp_state_sub = rospy.Subscriber('/exploration/state', ExplorationState, self.exp_state_cb)
        self.cmd_sub = rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.cmd_cb)

        self.states = InterfaceStates()
        self.interface_state = -1

        self.px4_cmd = PositionTarget()
        self.px4_cmd.header.frame_id = "map"
        self.px4_cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # self.px4_cmd.type_mask = PositionTarget.IGNORE_YAW_RATE

        # self.exp_state = ExplorationState().INIT

    def cmd_cb(self, msg):
        # msg = PositionCommand()
        # Position
        self.px4_cmd.position.x = msg.position.x
        self.px4_cmd.position.y = msg.position.y
        self.px4_cmd.position.z = msg.position.z
        # Velocity
        self.px4_cmd.velocity.x = msg.velocity.x
        self.px4_cmd.velocity.y = msg.velocity.y
        self.px4_cmd.velocity.z = msg.velocity.z
        # Acceleration
        self.px4_cmd.acceleration_or_force.x = msg.acceleration.x
        self.px4_cmd.acceleration_or_force.y = msg.acceleration.y
        self.px4_cmd.acceleration_or_force.z = msg.acceleration.z
        # Yaw
        self.px4_cmd.yaw = msg.yaw
        # Yaw rate
        self.px4_cmd.yaw_rate = msg.yaw_dot


    def exp_state_cb(self, msg):
        if msg.state <= ExplorationState.WAIT_TRIGGER:
            self.interface_state = self.states.INIT_READY

        if msg.state > ExplorationState.WAIT_TRIGGER and msg.state != ExplorationState.FINISH:
            self.interface_state = self.states.ACTIVE

        if msg.state == ExplorationState.FINISH:
            self.interface_state = self.states.DONE

    def get_control_command(self):
        return self.px4_cmd

    def set_active(self):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        # Add a dummy waypoint
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        pose.pose.orientation = Quaternion(0, 0, 0, 1)
        path.poses.append(pose)
        self.trigger_pub.publish(path)
        rospy.loginfo("[ExplorationInterface] Exploration Triggered")
        self.interface_state = self.states.ACTIVE

    def get_current_state(self):
        return self.interface_state


