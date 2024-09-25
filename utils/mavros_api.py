#!/usr/bin/python
import numpy as np
import math
import rospy
import sys

from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_matrix

from mavros_msgs.msg import State, ParamValue, PositionTarget
from mavros_msgs.srv import (CommandBool, CommandBoolRequest, CommandBoolResponse,
                            SetMode, SetModeRequest,SetModeResponse,
                            CommandTOL, CommandTOLRequest,
                            ParamSet, ParamSetRequest, ParamSetResponse,
                            ParamGet, ParamGetRequest, ParamGetResponse)

from copy import deepcopy
from std_msgs.msg import ColorRGBA, Bool
from geometry_msgs.msg import Vector3, PoseStamped, PoseArray, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

# System state class to keep track of the SW state
class APISystemState():

    def __init__(self) -> None:

        self.STATE_WAIT_INIT           =   0
        self.STATE_WAIT_POSITION_FIX   =   1
        self.STATE_READY               =   2
        self.STATE_FLYING              =   3
        self.STATE_MISSION_ACTIVE      =   4
        self.STATE_MISSION_DONE        =   5
        self.STATE_LANDING             =   6
        self.STATE_GROUNDED            =   7
        self.STATE_WAIT_MISSION        =   8
        self.STATE_MISSION_READY       =   9
        self.STATE_WAIT_PARAM_CFG      =   10
        self.STATE_LOCKDOWN            =   11

        self.state_to_str = {
                               0: "STATE_WAIT_INIT"           ,
                               1: "STATE_WAIT_POSITION_FIX"   ,
                               2: "STATE_READY"               ,
                               3: "STATE_FLYING"              ,
                               4: "STATE_MISSION_ACTIVE"      ,
                               5: "STATE_MISSION_DONE"        ,
                               6: "STATE_LANDING"             ,
                               7: "STATE_GROUNDED"            ,
                               8: "STATE_WAIT_MISSION"        ,
                               9: "STATE_MISSION_READY"       ,
                              10: "STATE_WAIT_PARAM_CFG"      ,
                              11: "STATE_LOCKDOWN"            ,
                            }

    def check_state(self, state):
        return state in self.state_to_str.keys()



class MavrosAPI():

    def __init__(self):
        rospy.loginfo("[MavrosAPI] Mavros Interface loaded")

        # Constants
        pt = PositionTarget()

        self.POSE_SP_MASK = (pt.IGNORE_VX+pt.IGNORE_VY+pt.IGNORE_VZ
                            +pt.IGNORE_AFX+pt.IGNORE_AFY+pt.IGNORE_AFZ
                            +pt.IGNORE_YAW_RATE)

        self.VEL_ONLY_SP_MASK = (pt.IGNORE_PX+pt.IGNORE_PY+pt.IGNORE_PZ
                            +pt.IGNORE_AFX+pt.IGNORE_AFY+pt.IGNORE_AFZ
                            +pt.IGNORE_YAW+pt.IGNORE_YAW_RATE)

        self.VEL_YAW_SP_MASK = (pt.IGNORE_PX+pt.IGNORE_PY+pt.IGNORE_PZ
                            +pt.IGNORE_AFX+pt.IGNORE_AFY+pt.IGNORE_AFZ
                            +pt.IGNORE_YAW_RATE)

        self.POSE_VEL_SP_MASK = (pt.IGNORE_AFX+pt.IGNORE_AFY+pt.IGNORE_AFZ
                                +pt.IGNORE_YAW_RATE)

        self.POSE_VEL_ACC_SP_MASK = (pt.IGNORE_YAW_RATE)

        # Some variable definitions
        self.flag_land_engaged = False #Flag to indicate if LAND mode engaged
        self.flag_got_pos_fix = False
        self.flag_fcu_ready = False
        self.flag_wps_displayed = False
        self.flag_collision = False
        self.flag_en_avoid = False
        self.flag_takeoff_done = False
        self.flag_vio_failed = False
        self.flag_param_checks_failed = False

        self.land_offset_x = rospy.get_param('~land_offset_x', -0.25)
        self.land_offset_y = rospy.get_param('~land_offset_y', 0.0)
        self.land_speed = rospy.get_param('~land_speed', 0.25)
        self.prec_land_final_alt = rospy.get_param('~prec_land_critical_hgt', 0.50)
        self.wp_hold_time = rospy.get_param('~waypoint_hold_time', 0.50)
        self.vio_failure_timeout = rospy.get_param('~vio_failure_timeout', 2.0)
        self.flag_en_vio_failsafe = rospy.get_param('~enable_vio_failsafe', False)
        self.flag_en_dyn_avoid = rospy.get_param('~enable_dynamic_avoidance', False)
        self.dyn_avoid_gain = rospy.get_param('~dyn_avoid_gain', 1.75)
        self.wp_exec_mode =   rospy.get_param('~exec_mode', 0)
        self.vel_ctrl_gain =  rospy.get_param('~vel_ctrl_gain',0.35) #Proportional gain for the velocity controller
        self.flag_align_yaw_bf_move = rospy.get_param('~align_yaw_before_move', True)
        self.flag_en_wall_follow = rospy.get_param('~enable_wall_following', True)
        self.flag_en_param_checks = rospy.get_param('~enable_px4_param_checks', False)

        # velocity FF is mandatory when using wall-following
        if self.flag_en_wall_follow:
            self.wp_exec_mode = int(1)

        self.curr_mav_pose = PoseStamped()
        self.curr_pose_target_sp = PositionTarget()
        self.waypoints = [] #list of wps in [x, y, z, yaw] format
        self.curr_mav_state = State()
        self.px4_param_dict = {}
        self.px4_param_checklist = {}
        self.curr_vel = np.zeros([3,1])
        self.curr_pos = np.zeros([3,1])
        self.curr_yaw = 0.0
        self.obs_det_pos = np.zeros([3,1]) #Position of the drone when the obstacle was detected within the critical limit
        self.prev_vio_fail_stamp = rospy.Time.now()
        self.api_states = APISystemState()
        self.curr_api_state = self.api_states.STATE_GROUNDED

        # Parameters
        self.wp_radius = 0.25 # Desired waypoint radius
        self.flag_en_offset = False # Enable 90deg yaw offset (only for VOXL)
        self.max_hor_speed =  0.75
        self.max_vert_speed = 0.75
        self.takeoff_alt = 1.0

        max_vel_lim = np.array([self.max_hor_speed, self.max_vert_speed])
        self.max_vel_mag = np.linalg.norm(max_vel_lim)

        # Publishers
        self.local_pose_pub = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=0)
        self.local_pose_vel_pub = rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,queue_size=0)
        self.waypoint_marker_pub = rospy.Publisher('/waypoint_markers',MarkerArray,queue_size=0, latch=True)

        # Subscribers
        rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.mav_pose_cb)
        rospy.Subscriber('/mavros/state',State,self.mav_state_cb)

        self.flag_wp_ready = False # Flag to indicate successful parsing of the .txt file

    # Intialise connection with FCU and setup some params
    def init_fcu(self):
        self.set_api_state(self.api_states.STATE_WAIT_INIT)
        # Make sure MAVROS and PX4 SITL Handshake otherwise sleep
        init_time = rospy.Time.now()

        while not rospy.is_shutdown() and not self.curr_mav_state.connected:
            rospy.logwarn("[MavrosAPI] Waiting for FCU connection...")
            rospy.sleep(1.0)
            if (rospy.Time.now()-init_time) > rospy.Duration(20.0):
                rospy.logerr("[MavrosAPI] Connection timed out!")
                rospy.logerr("[MavrosAPI] Error connecting to FCU")
                return False
        self.set_api_state(self.api_states.STATE_WAIT_POSITION_FIX)
        rospy.loginfo("[MavrosAPI] FCU connected.")

        # Wait for FCU to get fully initialised and get position fix
        while not rospy.is_shutdown() and not (self.flag_fcu_ready or self.flag_got_pos_fix):
            rospy.logwarn("[MavrosAPI] Waiting for position fix...")
            rospy.sleep(1.0)

        rospy.loginfo("[MavrosAPI] Got position data!")
        self.set_api_state(self.api_states.STATE_WAIT_PARAM_CFG)
        # Sleep for some time
        rospy.sleep(1.0)

        # Setup necessary PX4 Parameters
        for param in self.px4_param_dict.keys():
            self.set_param(param, self.px4_param_dict[param])

        # Check param values (only if enabled by user)
        if self.flag_en_param_checks:
            for param in self.px4_param_checklist.keys():

                value = self.get_param(param)

                if value == None or value != self.px4_param_checklist[param]:
                    if value!=None:
                        rospy.logerr("[MavrosAPI] PX4 parameter "+param+" not matching nominal value")
                    else:
                        rospy.logerr("[MavrosAPI] PX4 parameter "+param+" not found!")

                    self.flag_param_checks_failed = True
                    return False
                else:
                    rospy.loginfo("[MavrosAPI] PX4 parameter "+param+" matching nominal value!")

        self.set_api_state(self.api_states.STATE_READY)
        return True


    # Setter Methods
    def set_takeoff_alt(self, alt):
        self.takeoff_alt = alt

    def set_wp_radius(self, rad):
        self.wp_radius = rad

    def set_speed_limits(self, hor_lim, vert_lim):
        self.max_hor_speed = hor_lim
        self.max_vert_speed = vert_lim
        max_vel_lim = np.array([self.max_hor_speed, self.max_vert_speed])
        self.max_vel_mag = np.linalg.norm(max_vel_lim)

    def set_vel_ctrl_gain(self, k):
        self.vel_ctrl_gain = k

    def set_px4_param_list(self, dict):
        self.px4_param_dict = dict

    def set_px4_param_checklist(self, dict):
        self.px4_param_checklist = dict

    def toggle_voxl_offset(self, flag):
        self.flag_en_offset = flag

    # set waypoints to be tracked
    def set_wps(self, wps):
        self.waypoints = deepcopy(wps)
        self.flag_wp_ready = True
        self.display_wps()

    def clear_wps(self):
        self.waypoints = []
        self.flag_wp_ready = False
        self.flag_wps_displayed = False

    # Getter methods
    def get_curr_pos(self):
        return deepcopy(self.curr_pos)

    def get_curr_vel(self):
        return deepcopy(self.curr_vel)

    def get_curr_yaw(self):
        return deepcopy(self.curr_yaw)

    def get_curr_mode(self):
        return self.curr_mav_state.mode

    # Function to set PX4 parameters
    def set_param(self,param_id, val):
        rospy.wait_for_service('mavros/param/set')
        srvClient = rospy.ServiceProxy('mavros/param/set',ParamSet)

        int_val = 0
        real_val = 0.0

        if type(val)==int:
            int_val = val
        elif type(val)==float:
            real_val = val

        srvReq = ParamSetRequest()
        srvReq.param_id = param_id
        srvReq.value = ParamValue(int_val,real_val)

        try:
            srvResp = srvClient(srvReq)
            if srvResp.success:
                rospy.loginfo('[MavrosAPI] PX4 parameter '+param_id+ ' set successfully!')
            else:
                rospy.logerr('[MavrosAPI] Setting PX4 parameter '+param_id+ ' failed!')

            return srvResp.success
        except rospy.ServiceException as e:
            rospy.loginfo(e)
            return False


    def get_param(self, param_id):
        rospy.wait_for_service('mavros/param/get')
        srvClient = rospy.ServiceProxy('mavros/param/get',ParamGet)

        srvReq = ParamGetRequest()
        srvReq.param_id = param_id

        try:
            srvResp = srvClient(srvReq)
            if srvResp.success:
                rospy.loginfo('[MavrosAPI] PX4 parameter '+param_id+ ' retrieved successfully!')
                return srvResp.value.integer + srvResp.value.real
            else:
                rospy.logerr('[MavrosAPI] Retreiving PX4 parameter '+param_id+ ' failed!')

            return None
        except rospy.ServiceException as e:
            rospy.loginfo(e)
            return None


    # MAV Current State Callback
    def mav_state_cb(self,msg):

        self.curr_mav_state = msg

        if not self.flag_fcu_ready and msg.guided:
            self.flag_fcu_ready = True


    # MAV Current Pose Callback - Updates the current pose of the MAV
    def mav_pose_cb(self,msg):

        if not self.flag_got_pos_fix:
            self.flag_got_pos_fix = True

        if self.flag_en_offset:

            # convert position to fixed frame
            self.curr_mav_pose.pose.position.x = msg.pose.position.y
            self.curr_mav_pose.pose.position.y = -msg.pose.position.x
            self.curr_mav_pose.pose.position.z = msg.pose.position.z

            # convert yaw to fixed frame
            yaw = self.quat_to_yaw(msg.pose.orientation)
            yaw -= math.radians(90)

            self.curr_mav_pose.pose.orientation = self.yaw_to_quat(yaw)

        else:
            self.curr_mav_pose = msg

        self.curr_pos[0] = self.curr_mav_pose.pose.position.x
        self.curr_pos[1] = self.curr_mav_pose.pose.position.y
        self.curr_pos[2] = self.curr_mav_pose.pose.position.z
        self.curr_yaw = self.quat_to_yaw(self.curr_mav_pose.pose.orientation)


    # Use PX4 in-built takeoff mode
    def do_takeoff(self):
        if self.set_mode('AUTO.TAKEOFF'):
            rospy.loginfo('[MavrosAPI] Started takeoff...')
            self.set_api_state(self.api_states.STATE_FLYING)
            while (not rospy.is_shutdown() and
                   not abs(self.curr_mav_pose.pose.position.z - self.takeoff_alt) < 0.50):
                rospy.sleep(1.0)
            rospy.loginfo("[MavrosAPI] Takeoff completed!")
            self.set_api_state(self.api_states.STATE_WAIT_MISSION)
            self.flag_takeoff_done = True
            return True

        rospy.logerr("[MavrosAPI] Takeoff request rejected!")
        return False


    # Function to land the drone
    def do_land(self):
        rospy.loginfo("[MavrosAPI] Initiating landing...")
        while not rospy.is_shutdown() and not self.flag_land_engaged:
            self.flag_land_engaged = self.set_mode('AUTO.LAND')
            rospy.sleep(0.50)

        self.set_api_state(self.api_states.STATE_LANDING)

        while not rospy.is_shutdown() and self.curr_mav_state.armed:
            rospy.sleep(1.0)

        self.set_api_state(self.api_states.STATE_GROUNDED)


        rospy.loginfo('[MavrosAPI] Landed successfuly!')

    # Function to change flight modes
    def set_mode(self,mode_id):
        # Mode change service client
        modeClient = rospy.ServiceProxy('/mavros/set_mode',SetMode)
        modeReq = SetModeRequest()
        modeResp = SetModeResponse()

        modeReq.custom_mode = mode_id
        modeReq.base_mode = 0

        # Request mode change
        try:
            rospy.loginfo_throttle(2.0,'[MavrosAPI] Attempting flight mode change to '+ mode_id)
            modeResp = modeClient(modeReq)
            return modeResp.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr('[MavrosAPI] Mode change to '+ mode_id +' failed! %s'%e)
            return False

    # Function to switch the MAV to Offboard and hold current position
    def set_offboard(self):
        rospy.loginfo_throttle(2.0, "[MavrosAPI] Switching to Offboard mode...")

        rate = rospy.Rate(50)
        pose_sp = PositionTarget()

        while not rospy.is_shutdown():

            # Populating the local pose SP message
            pose_sp.header.frame_id = 'map'
            pose_sp.header.stamp = rospy.Time.now()
            pose_sp.coordinate_frame = pose_sp.FRAME_LOCAL_NED
            pose_sp.type_mask = self.POSE_SP_MASK

            # Publish the local pose SP corressponding to the current MAV pose
            pose_sp.position = self.curr_mav_pose.pose.position
            pose_sp.yaw =self.quat_to_yaw(self.curr_mav_pose.pose.orientation)

            self.publish_setpoints(pose_sp)

            if self.curr_mav_state.mode=='OFFBOARD':
                rospy.loginfo("[MavrosAPI] Switched to Offboard mode!")
                self.set_api_state(self.api_states.STATE_MISSION_READY)
                break

            else:
                self.set_mode('OFFBOARD')

            rate.sleep()

        return True

    # Convenience function for calling the Arming service
    def set_arming(self):
        if self.curr_mav_state.armed:
            return True

        srvClient = rospy.ServiceProxy('mavros/cmd/arming',CommandBool)
        srvReq = CommandBoolRequest()
        srvReq.value = True

        srvResp = CommandBoolResponse()
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            srvResp = srvClient(srvReq)
            if srvResp.success:
                rospy.loginfo('[MavrosAPI] Vehicle Armed and Ready!')
            else:
                rospy.logerr("[MavrosAPI] Arming call failed")

            return srvResp.success
        except rospy.ServiceException as e:
            rospy.logerr("[MavrosAPI] Arming call failed: %s"%e)
            return False

    # Convenience function to obtain the euclidean distance b/w MAV and current WP
    def dist_to_wp(self, curr_wp):
        dx = curr_wp[0] - self.curr_pos[0]
        dy = curr_wp[1] - self.curr_pos[1]
        dz = curr_wp[2] - self.curr_pos[2]

        return np.linalg.norm([dx,dy,dz])

    # New distance metric for wall-follow method (move Y-coordinate checks outside)
    def dist_to_wp_xz(self, curr_wp):
        dx = curr_wp[0] - self.curr_pos[0]
        dy = 0.0
        dz = curr_wp[2] - self.curr_pos[2]

        return np.linalg.norm([dx,dy,dz])


    # Function to track the waypoints in a Point-to-Point (P2P) manner
    def execute_wps(self):
        self.set_api_state(self.api_states.STATE_MISSION_ACTIVE)
        rospy.loginfo("[MavrosAPI] Starting WP execution...")
        rate = rospy.Rate(20)
        num_wps = len(self.waypoints)
        ctr=0

        curr_wp = self.waypoints[0]

        while(not(rospy.is_shutdown())):

            if self.curr_api_state !=self.api_states.STATE_MISSION_ACTIVE or self.curr_mav_state.mode!=self.curr_mav_state.MODE_PX4_OFFBOARD:
                rospy.logerr("[MavrosAPI] WP execution halted! System in emergency state or under manual control!")
                self.set_api_state(self.api_states.STATE_LOCKDOWN)
                return False

            # If MAV close enough to the current WP, update the current WP
            if not self.flag_en_wall_follow:
                if self.dist_to_wp(curr_wp) < self.wp_radius and ctr<num_wps-1:
                    rospy.loginfo("[MavrosAPI] Reached WP "+str(ctr+1)+" of "+str(num_wps))
                    self.offb_hold(curr_wp, self.wp_hold_time)
                    ctr+=1
                    curr_wp = self.waypoints[ctr]
            # If wall following enabled use the different WP distance metric
            else:
                if self.dist_to_wp_xz(curr_wp) < self.wp_radius and abs(self.curr_pos[1]-curr_wp[1])<1.0 and ctr<num_wps-1:
                    rospy.loginfo("[MavrosAPI] Reached WP "+str(ctr+1)+" of "+str(num_wps))
                    self.offb_hold(curr_wp, self.wp_hold_time)
                    ctr+=1
                    curr_wp = self.waypoints[ctr]

            yaw_error = self.curr_yaw-curr_wp[3]

            # Wrap error b/w pi and -pi
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi

            while yaw_error <= -math.pi:
                yaw_error += 2 * math.pi

            # If yaw not algined hold at current position and align the yaw before moving again
            if self.flag_align_yaw_bf_move and abs(yaw_error)>0.25:
                msg = PositionTarget()
                msg.coordinate_frame = msg.FRAME_LOCAL_NED
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'map'
                msg.type_mask = self.POSE_SP_MASK

                msg.position.x = self.curr_pos[0]
                msg.position.y = self.curr_pos[1]
                msg.position.z = self.curr_pos[2]
                msg.yaw = curr_wp[3]
                rospy.logwarn_throttle(1.0, "[MavrosAPI] Aligning yaw...")
            # Add WP pose setpoints only if yaw is already aligned
            else:
                msg = PositionTarget()
                msg.coordinate_frame = msg.FRAME_LOCAL_NED
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'map'
                msg.type_mask = self.POSE_SP_MASK

                msg.position.x = curr_wp[0]
                msg.position.y = curr_wp[1]
                msg.position.z = curr_wp[2]
                msg.yaw = curr_wp[3]


            # If velocity Feed-forward enabled, compute velocity commands as well
            if self.wp_exec_mode==1:
                # Add vel setpoints only if yaw is already aligned
                if self.flag_align_yaw_bf_move and abs(yaw_error)>0.25:
                    rospy.logwarn_throttle(1.0, "[MavrosAPI] Aligning yaw...")
                else:
                    vel_sp = self.get_vel_ctrl(curr_wp)
                    # print(vel_sp)

                    msg.type_mask = self.POSE_VEL_SP_MASK
                    msg.velocity.x = vel_sp[0]
                    msg.velocity.y = vel_sp[1]
                    msg.velocity.z = vel_sp[2]

            # Publish the WP as a position target SP
            self.publish_setpoints(msg)
            self.curr_pose_target_sp = msg

            if not self.flag_en_wall_follow:
                if self.dist_to_wp(self.waypoints[-1])<self.wp_radius and not (ctr==0 and not num_wps==1):
                    rospy.loginfo("[MavrosAPI] Reached WP "+str(num_wps)+" of "+str(num_wps))
                    # Always hold at the last WP
                    self.offb_hold(curr_wp, 3.0)
                    rospy.loginfo("[MavrosAPI] Done with WP tracking!")
                    self.set_api_state(self.api_states.STATE_MISSION_DONE)
                    return True
            else:
                # If wall following enabled use the different WP distance metric
                if (self.dist_to_wp_xz(self.waypoints[-1])<self.wp_radius and abs(self.curr_pos[1]-self.waypoints[-1][1])<1.0) and not (ctr==0 and not num_wps==1):
                    rospy.loginfo("[MavrosAPI] Reached WP "+str(num_wps)+" of "+str(num_wps))
                    # Always hold at the last WP
                    self.offb_hold(curr_wp, 3.0)
                    rospy.loginfo("[MavrosAPI] Done with WP tracking!")
                    self.set_api_state(self.api_states.STATE_MISSION_DONE)
                    return True

            rate.sleep()


    # simple proportional controller
    def get_vel_ctrl(self, curr_wp):
        # curr_pose_vec = self.from_pose_msg(self.curr_mav_pose.pose)

        dx = curr_wp[0] - self.curr_pos[0]
        dy = curr_wp[1] - self.curr_pos[1]
        dz = curr_wp[2] - self.curr_pos[2]

        pos_err = np.array([dx, dy, dz])

        vel_sp = self.vel_ctrl_gain*(pos_err)

        vel_sp_mag = np.linalg.norm(vel_sp)

        if vel_sp_mag > self.max_vel_mag:
            vel_sp_dir = vel_sp/vel_sp_mag
            vel_sp = self.max_vel_mag*vel_sp_dir

        # print(vel_sp)
        return vel_sp


    def to_pose_msg(self, wp):
        pose = Pose()

        pose.position.x = wp[0]
        pose.position.y = wp[1]
        pose.position.z = wp[2]
        pose.orientation = self.yaw_to_quat(wp[3])

        return pose

    def from_pose_msg(self, pose):
        wp = np.zeros([4,1])

        wp[0] = pose.position.x
        wp[1] = pose.position.y
        wp[2] = pose.position.z
        wp[3] = self.quat_to_yaw(pose.orientation)

        # print(wp)
        return wp

    # Function to hold the drone at desired position in Offboard mode
    def offb_hold(self,curr_wp ,secs):

        init_time = rospy.Time.now()
        hold_pose = PositionTarget()
        hold_pose.header.frame_id = 'map'
        hold_pose.coordinate_frame = hold_pose.FRAME_LOCAL_NED
        hold_pose.type_mask = self.POSE_SP_MASK

        hold_pose.position.x = curr_wp[0]
        hold_pose.position.y = curr_wp[1]
        hold_pose.position.z = curr_wp[2]
        hold_pose.yaw        = curr_wp[3]

        while(rospy.Time.now()-init_time < rospy.Duration(secs)):
            hold_pose.header.stamp = rospy.Time.now()
            self.publish_setpoints(hold_pose)

    def publish_setpoints(self, pose_vel_sp):
        sp = PositionTarget()
        sp = pose_vel_sp

        #  If offset is enabled do necessary transforms
        if self.flag_en_offset:
            x = -sp.position.y
            y =  sp.position.x
            vx = -sp.velocity.y
            vy = sp.velocity.x
            yaw = sp.yaw

            sp.position.x = x
            sp.position.y = y
            sp.velocity.x = vx
            sp.velocity.y = vy

            yaw += math.radians(90)
            sp.yaw = yaw

        # Publish the setpoints only when not in emergency/manual control
        if self.curr_api_state != self.api_states.STATE_LOCKDOWN:
            self.local_pose_vel_pub.publish(sp)

    def quat_to_yaw(self, q):
        quat = [q.x, q.y, q.z, q.w]
        eul = euler_from_quaternion(quat,'rzyx')
        return eul[0]

    def yaw_to_quat(self,yaw):
        q = quaternion_from_euler(yaw,0.0,0.0,'rzyx')

        return Quaternion(q[0], q[1], q[2], q[3])


    # Populate the Marker Array msg for Visualization of the WPs in RViz
    def display_wps(self):
        m = Marker()
        marker_msg = MarkerArray()
        ctr = 0
        wp_num = 1
        for wp in self.waypoints:
            # WP marker
            m.id = ctr
            m.type = Marker.CUBE
            m.action = Marker.MODIFY
            m.color = ColorRGBA(1,0.2,0,1)
            m.scale = Vector3(0.1,0.1,0.1)
            m.header.frame_id = 'map'
            m.header.stamp= rospy.Time.now()
            m.pose = self.to_pose_msg(wp)
            marker_msg.markers.append(deepcopy(m))
            ctr+=1

            # Text Marker
            m = Marker()
            m.id = ctr
            m.action = m.ADD
            m.header.frame_id = 'map'
            m.header.stamp= rospy.Time.now()
            m.color = ColorRGBA(1,1,1,1)
            m.type = m.TEXT_VIEW_FACING
            m.scale.z = 0.20
            m.pose = self.to_pose_msg(wp)
            m.pose.position.z += 0.10
            m.text = "WP "+str(wp_num)
            marker_msg.markers.append(deepcopy(m))
            ctr+=1

            wp_num+=1

        # Line Strip Marker
        m = Marker()
        m.id = ctr
        m.type = Marker.LINE_STRIP
        m.action = Marker.MODIFY
        m.color = ColorRGBA(1.0,0,0,1)
        m.scale = Vector3(0.05,0.0,0.0)
        m.header.frame_id = 'map'
        m.header.stamp= rospy.Time.now()
        m.points = [self.to_pose_msg(p).position for p in self.waypoints]
        marker_msg.markers.append(deepcopy(m))

        self.waypoint_marker_pub.publish(marker_msg)

    def set_api_state(self, state):
        if not self.api_states.check_state(state):
            rospy.logerr("[MavrosAPI] Unknown API state!")
            return

        if self.curr_api_state == state:
            return

        rospy.logwarn("[MavrosAPI] API state changed from "+self.api_states.state_to_str[self.curr_api_state]+" >>> "+self.api_states.state_to_str[state])
        self.curr_api_state = state
