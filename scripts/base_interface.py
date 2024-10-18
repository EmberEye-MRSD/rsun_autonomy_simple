import rospy
from mavros_msgs.msg import PositionTarget
from interface_state import InterfaceStates

class BaseInterface:
      def __init__(self):
         # self.interface_state = InterfaceStates.PRE_INIT
         self.cmd_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
         # self.interface_state = InterfaceStates.PRE_INIT

      def set_active(self):
         pass

      def get_control_command(self):
         pass

      def get_current_state(self):
         return self.interface_state