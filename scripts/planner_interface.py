import rospy
from geometry_msgs.msg import PoseStamped,Quaternion
from nav_msgs.msg import Odometry, Path
from mavros_msgs.msg import PositionTarget
from quadrotor_msgs.msg import PositionCommand
from exploration_manager.msg import ExplorationState
from autonomous_flight.msg import SystemStatus
from interface_state import InterfaceStates
from base_interface import BaseInterface
from tracking_controller.msg import Target


class PlannerInterface(BaseInterface):
    
    def __init__(self):
        super().__init__()

        # Planner Trigger
        rospy.loginfo("[PlannerInterface] Loaded!")
        self.trigger_pub = rospy.Publisher('/p2p_nav/goal', PoseStamped, queue_size=10)
        self.planner_state_sub = rospy.Subscriber('/navigation/trajectory_status', SystemStatus, self.planner_state_cb)
        self.trigger_rviz_sub = rospy.Subscriber('/p2p_nav/trigger', PoseStamped, self.trigger_rviz_cb)
        self.cmd_sub = rospy.Subscriber('/autonomous_flight/target_state', Target, self.cmd_cb)


        self.states = InterfaceStates()
        self.interface_state = -1

        self.goal_received = False
        self.send_trigger = False

        self.px4_cmd = PositionTarget()
        self.px4_cmd.header.frame_id = "map"
        self.px4_cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.px4_cmd.type_mask = PositionTarget.IGNORE_YAW_RATE

        self.entered_exploration_area = False

    ## receives pose from target state which originally went to tracking controller of p2p module
    def cmd_cb(self, msg):

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

        # If x position is more than a set value (convert it to rosparam later)
        if msg.position.x > 11 and not self.entered_exploration_area:
            rospy.loginfo("[PlannerInterface] Entered Exploration Area!")
            self.entered_exploration_area = True

    def get_control_command(self):
        return self.px4_cmd
    
    def get_current_state(self):
        return self.interface_state

    ## triggers the activation function
    def trigger_rviz_cb(self, msg):
        
        ## check if msg not empty
        if msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0:
            rospy.loginfo("[PlannerInterface] Empty Trigger!")
            self.goal_received = False
        else:
            if (self.interface_state == self.states.DONE):
                rospy.loginfo("[PlannerInterface] Another goal received")
            else:
                rospy.loginfo("[PlannerInterface] Trigger Received!")
            self.goal_received = True
            
            self.set_active(msg)

    def planner_state_cb(self, msg):
        if msg.state == SystemStatus.PRE_INIT:
            self.interface_state = self.states.PRE_INIT

        if msg.state == SystemStatus.INIT_READY:
            self.interface_state = self.states.INIT_READY

        if msg.state == SystemStatus.ACTIVE:
            self.interface_state = self.states.ACTIVE

        if msg.state == SystemStatus.DONE:
            self.interface_state = self.states.DONE

        if msg.state == SystemStatus.FAILURE:
            self.interface_state = self.states.FAILURE

    ## activation function: sends the goal to the planner
    def set_active(self, msg):
        trigger = PoseStamped()
        trigger.header.frame_id = msg.header.frame_id
        trigger.pose.position.x = msg.pose.position.x
        trigger.pose.position.y = msg.pose.position.y
        trigger.pose.position.z =  msg.pose.position.z
        trigger.pose.orientation = msg.pose.orientation

        if (self.send_trigger):
            self.trigger_pub.publish(trigger)
            rospy.loginfo("[PlannerInterface] Trigger Sent!")
            self.send_trigger = False
            self.goal_received = False

        ## do we need to set state to active explicitly here?


# if __name__ == "__main__":
#     rospy.init_node('planner_interface')
#     planner = PlannerInterface()
#     rate = rospy.Rate(10)
    

        



