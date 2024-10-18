#!/usr/bin/python3
import numpy as np
import math
import rospy, csv
import sys
import json

sys.path.append('../utils')
from mavros_api import MavrosAPI
from interface_state import InterfaceStates
from exploration_interface import ExplorationInterface
from planner_interface import PlannerInterface

class ManagerState():
    def __init__(self):
        self.GROUNDED = 0
        self.TAKEOFF = 1
        self.HOVER = 2
        self.ACTIVE_STANDBY = 3
        self.ACTIVE_P2P = 4
        self.ACTIVE_EXP = 5
        self.LANDING = 6
        self.MISSION_DONE = 7

    def to_string(self, state):
        if state == self.GROUNDED:
            return "GROUNDED"
        elif state == self.TAKEOFF:
            return "TAKEOFF"
        elif state == self.HOVER:
            return "HOVER"
        elif state == self.ACTIVE_STANDBY:
            return "ACTIVE_STANDBY"
        elif state == self.ACTIVE_P2P:
            return "ACTIVE_P2P"
        elif state == self.ACTIVE_EXP:
            return "ACTIVE_EXP"
        elif state == self.LANDING:
            return "LANDING"
        elif state == self.MISSION_DONE:
            return "DONE"
        else:
            return "UNKNOWN"

STATE = ManagerState()

MODE_DICT = {0: "WAYPOINT", 1: "P2P_NAVIGATION", 2: "EXPLORATION"}

class AutonomyManager():

    def __init__(self):
        rospy.loginfo("Waypoint Tracking Node Started!")

        # ROS params
        self.wp_radius              = rospy.get_param('~wp_radius', 0.25)
        self.flag_yaw_hold          = rospy.get_param('~yaw_hold',False) # Parameter to enable constant Yaw direction otherwise use Auto align towards next WP
        self.takeoff_alt            = rospy.get_param('~takeoff_height', 1.0) # Desired takeoff height
        self.max_hor_speed          = rospy.get_param('~speed_hor_max', 0.75)
        self.max_vert_speed         = rospy.get_param('~speed_vert_max', 0.75)
        self.flag_en_offset         = rospy.get_param('~enable_voxl_offset',True)
        self.wp_mode                = rospy.get_param('~wp_mode',0) #0: pt-pt, 1: grid-planning
        self.flag_en_avoid          = rospy.get_param('~enable_avoidance', False)
        self.flag_en_prec_land      = rospy.get_param('~enable_prec_land', True)
        self.flag_face_rack         = rospy.get_param('~enable_rack_facing', False)
        self.grid_scan_mode         = rospy.get_param('~grid_scan_mode', 'HOR')
        self.num_grid_levels        = rospy.get_param('~num_grid_levels', 5)
        self.px4_param_checklist    = rospy.get_param('~px4_param_checklist')
        self.nav_hdg_offset         = rospy.get_param('~heading_offset_deg', 0.0)
        self.use_extern_yaw         = rospy.get_param('~read_extern_yaw', False)
        # self.nav_mode               = rospy.get_param('~nav_mode', 2) # 0: Waypoints, 1: Exploration, 2: P2P Navigation
        self.nav_mode               = 0
        # Some flags
        self.flag_wp_ready = False # Flag to indicate successful parsing of the .txt file

        # PX4 Params to be set
        self.px4_param_dict = {
            # Auto-takeoff altitude
            'MIS_TAKEOFF_ALT': self.takeoff_alt,
            # horizontal speed limit
            'MPC_XY_VEL_MAX' : self.max_hor_speed,
            # # vertical speed limit (up)
            'MPC_Z_VEL_MAX_UP'  : self.max_vert_speed,
            # # vertical speed limit (down)
            'MPC_Z_VEL_MAX_DN'  : self.max_vert_speed,
            # limit max yaw rate to avoid VIO failure
            'MC_YAWRATE_MAX'    : 100.0,
            # limit horizontal acceleration for smoother flight
            'MPC_ACC_HOR_MAX'   : 5.50
        }

        # Instantiate the mavros API class
        self.api = MavrosAPI()

        # Configure API params
        self.api.toggle_voxl_offset(self.flag_en_offset)
        self.api.set_px4_param_list(self.px4_param_dict)
        self.api.set_px4_param_checklist(self.px4_param_checklist)
        self.api.set_speed_limits(self.max_hor_speed,self.max_vert_speed)
        self.api.set_wp_radius(self.wp_radius)
        self.api.set_takeoff_alt(self.takeoff_alt)

        # Current manager state
        self.sys_state = STATE.GROUNDED

        # Exploration Interface
        self.interface_dict = {
            1: PlannerInterface(),
            2: ExplorationInterface()
        }

        #  FSM loop
        self.fsm_timer = rospy.Timer(rospy.Duration(0.02), self.fsm_cb)

        self.P2P_again = False
        self.home_flag = False
        self.was_active = False

    def change_state(self, new_state):
        str = f"[AutonomyManager] State Change! [{STATE.to_string(self.sys_state)}] >>> [{STATE.to_string(new_state)}]"
        self.sys_state = new_state
        rospy.logwarn(str)


    # FSM callback
    def fsm_cb(self, event):

        rospy.logwarn_throttle(5.0, "[AutonomyManager] FSM Current State: " + STATE.to_string(self.sys_state))

        if self.sys_state == STATE.GROUNDED:
            success = self.do_hover()
            if success:
                self.change_state(STATE.HOVER)

        if self.sys_state == STATE.HOVER:
            if self.api.set_offboard():
                self.change_state(STATE.ACTIVE_STANDBY)

        if self.sys_state == STATE.ACTIVE_STANDBY:
            # Check if in OFFBOARD mode
            if self.api.get_curr_mode() == "OFFBOARD":
                self.change_state(STATE.ACTIVE_P2P)
            else:
                self.change_state(STATE.HOVER)

            # if self.sys_state ==

        if self.sys_state == STATE.ACTIVE_P2P:
            # self.nav_node = 1
            ## get planner interface object
            interface = self.interface_dict[1]

            ## If failure mode just land for now
            if interface.get_current_state() == InterfaceStates().FAILURE:
                rospy.logerr("[AutonomyManager] P2P Node Encountered a Failure!")
                # self.change_state(STATE.LANDING)

            if interface.get_current_state() == InterfaceStates().INIT_READY:
                if interface.goal_received:
                    # rospy.loginfo("[AutonomyManager] Trigger Approved!")
                    interface.send_trigger = True


            if interface.get_current_state() == InterfaceStates().ACTIVE:
                self.api.publish_setpoints(interface.get_control_command())
                if self.P2P_again:
                    self.was_active = True
            
            if interface.get_current_state() == InterfaceStates().DONE:
                # rospy.loginfo("[AutonomyManager] Inside P2P Done!")
                if interface.goal_received:
                    # rospy.loginfo("[AutonomyManager] Received Another goal!")
                    interface.send_trigger = True
                    if self.P2P_again:
                        self.home_flag = True
            #         interface.send_trigger = True

                if interface.entered_exploration_area and not self.P2P_again:
                    self.change_state(STATE.ACTIVE_EXP)

                if self.P2P_again and self.was_active and self.home_flag:
                    rospy.loginfo("[AutonomyManager] Mission Completed, Landing!")
                    self.change_state(STATE.LANDING)



            # if interface.get_current_state() == InterfaceStates().DONE:
            #     self.change_state(STATE.LANDING)
                    
        if self.sys_state == STATE.ACTIVE_EXP:
            self.nav_node = 2

            interface = self.interface_dict[2]

            if interface.get_current_state() == InterfaceStates().INIT_READY:
                interface.set_active()

            if interface.get_current_state() == InterfaceStates().ACTIVE:
                self.api.publish_setpoints(interface.get_control_command())

            if interface.get_current_state() == InterfaceStates().DONE:
                self.change_state(STATE.ACTIVE_P2P)
                self.P2P_again = True

            # if interface.get_current_state() == InterfaceStates().DONE:
            #     self.change_state(STATE.LANDING)

        if self.sys_state == STATE.LANDING:
            self.api.do_land()
            self.change_state(STATE.MISSION_DONE)


    # Main helper fn housing the core mission logic
    def do_hover(self):

        # Sequence of mission steps with respective error messages
        mission_steps = [
            (self.api.init_fcu, "[AutonomyManager] FCU initialization failed!"),
            (self.api.set_arming, "[AutonomyManager] Arming failed!"),
            (self.api.do_takeoff, "[AutonomyManager] Takeoff failed!")
        ]

        # Execute each step, return False if any step fails
        for step, error_msg in mission_steps:
            if not step():
                rospy.logerr(error_msg)
                return False

        return True

    # Function to parse the .txt file and obtain the Waypoints
    def read_wps(self):
        self.wps_file_dir = rospy.get_param("~wps_file_dir","../misc/waypoints.txt")

        wp_list = []

        with open(self.wps_file_dir) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            prev_row = []
            for row in csv_reader:
                line_count += 1

                # [x,y,z,yaw] format
                wp_x = float(row[0][1:])
                wp_y = float(row[1])
                wp_z = float(row[2])
                wp_yaw = math.radians(float(row[3][0:-1]))

                if self.flag_yaw_hold:
                    # If Yaw hold enabled, use the initial Yaw direction for all WPs
                    wp_yaw = self.api.get_curr_yaw()
                elif line_count==1 and not self.flag_yaw_hold:
                    if not self.use_extern_yaw:
                        curr_pos = self.api.get_curr_pos()
                        wp_yaw = math.atan2(float(row[1])-curr_pos[1],
                                            float(row[0][1:])-curr_pos[0])

                        wp_yaw += math.radians(self.nav_hdg_offset)
                else:
                    if not self.use_extern_yaw:
                        # If Yaw hold is not enabled, use the direction towards next WP as the MAV Yaw SP
                        wp_yaw = math.atan2(float(row[1])-float(prev_row[1]),
                                            float(row[0][1:])-float(prev_row[0][1:]))

                        wp_yaw += math.radians(self.nav_hdg_offset)

                wp_list.append([wp_x, wp_y, wp_z, wp_yaw])
                # Print info to the user
                rospy.loginfo('[AutonomyManager] WP[' + str(line_count) + '] >>  X: ' + str(wp_x) + ', Y: ' + str(wp_y)+ ', Z: ' + str(wp_z) + ' Yaw: '+ str(wp_yaw))
                prev_row = row
            rospy.loginfo('[AutonomyManager] Got ' + str(line_count) + ' Waypoints from the file!')

            return wp_list


if __name__ == '__main__':
    try:
        # Intialise the node
        rospy.init_node('autonomy_manager', anonymous=False)
        # Instantiate the Node class
        nh = AutonomyManager()
        # Do not terminate until user keyboard interrupt
        rospy.spin()
    except rospy.ROSInterruptException:
        pass