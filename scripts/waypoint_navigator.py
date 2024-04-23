#!/usr/bin/python3
import numpy as np
import math
import rospy, csv
import sys
import json

sys.path.append('../utils')
from grid_planner import GridPlanner
from mavros_api import MavrosAPI
from rviz_wp_interface import RVizWPInterface

class WPTrackerNode():

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

        # Some flags
        self.flag_wp_ready = False # Flag to indicate successful parsing of the .txt file

        # PX4 Params to be set
        self.px4_param_dict = {
            # Auto-takeoff altitude
            'MIS_TAKEOFF_ALT': self.takeoff_alt,
            # horizontal speed limit
            # 'MPC_XY_VEL_MAX' : self.max_hor_speed,
            # # vertical speed limit (up)
            # 'MPC_Z_VEL_MAX_UP'  : self.max_vert_speed,
            # # vertical speed limit (down)
            # 'MPC_Z_VEL_MAX_DN'  : self.max_vert_speed,
            # limit max yaw rate to avoid VIO failure
            'MC_YAWRATE_MAX'    : 30.0,
            # limit horizontal acceleration for smoother flight
            'MPC_ACC_HOR_MAX'   : 5.50
        }

        
        # Instantiate the mavros API class
        self.api = MavrosAPI()
        self.rviz_wpi = RVizWPInterface(self.takeoff_alt)

        # Configure API params
        self.api.toggle_voxl_offset(self.flag_en_offset)
        self.api.set_px4_param_list(self.px4_param_dict)
        self.api.set_px4_param_checklist(self.px4_param_checklist)
        self.api.set_speed_limits(self.max_hor_speed,self.max_vert_speed)
        self.api.set_wp_radius(self.wp_radius)
        self.api.set_takeoff_alt(self.takeoff_alt)

        # Main program logic
        self.run_mission()

    # Main helper fn housing the core mission logic
    def run_mission(self):
        # Initialise the handshake with FCU and configure Firmware params
        if self.api.init_fcu():
            # Use appropriate WP source as per user config
            if self.wp_mode==0:
                # Read WPs from external file
                self.waypoints = self.read_wps()
            elif self.wp_mode==1:
                # Generate Grid Scan WP list
                self.api.wp_exec_mode = 1
                self.waypoints = self.generate_grid_plan()
            else:
                # read from RViz input topic
                while not self.rviz_wpi.flag_term_recvd:
                    rospy.logwarn("[AutoNavigator] Waiting for RViz WPs...")
                    rospy.sleep(2.0)
                self.waypoints = self.rviz_wpi.wp_list
            # Send generated WPs to API class
            self.api.set_wps(self.waypoints)

            # Attempt ARMING the drone
            if self.api.set_arming():
                # If success, start takeoff to user-defined alt
                if self.api.do_takeoff():
                    # Sleep for some time
                    rospy.sleep(5.0)
                    # Attempt switching to OFFB mode
                    if self.api.set_offboard():
                        if self.api.execute_wps():
                        # use precision landing only when enabled
                            if self.flag_en_prec_land:
                                self.api.offb_prec_land()
                            else:
                                self.api.do_land()
            
            rospy.loginfo("[AutoNavigator] Mission logic ended!")
        else:
            rospy.logerr("[AutoNavigator] FCU intialization failed!")



    # Function to generate Grid plan WPs for a given rack pose and size
    def generate_grid_plan(self):
        self.rack_info_dir = rospy.get_param("~rack_info_dir","../misc/rack_info.json")
        
        with open(self.rack_info_dir, 'r') as f:
            data = json.load(f)

        racks = data['racks']

        wp_list = []

        for rack in racks:
            # print(rack["size"]["length"])
            grid_obj = GridPlanner()

            if self.grid_scan_mode=='HOR':
                grid_spacing = (rack["size"]["height"]-rack["shelf_start_hgt"])/(self.num_grid_levels-1)
            elif self.grid_scan_mode=='VERT':
                grid_spacing = (rack["size"]["length"])/(self.num_grid_levels-1)

            grid_obj.set_scan_area(height=rack["size"]["height"]-rack["shelf_start_hgt"], width=rack["size"]["length"])
            grid_obj.set_rack_depth(rack["size"]["depth"])
            grid_obj.set_scan_margin(margin_hor=0.0,margin_vert=0.0)
            grid_obj.set_grid_spacing(grid_spacing)
            grid_obj.set_num_pts_line(int(10))
            grid_obj.set_scan_dist(0.50)
            grid_obj.set_grid_layout(self.grid_scan_mode)
            grid_obj.set_shelf_start_hgt(rack["shelf_start_hgt"])
            grid_obj.set_rack_position(np.array(rack["pose"]["position"]))
            grid_obj.set_rack_yaw(math.radians(rack["pose"]["yaw"]))

            grid_obj.generate_plan()
            
            wps = grid_obj.get_plan()

            for i in range(len(wps)-1):
                wp = wps[i]
                if self.flag_face_rack or self.grid_scan_mode=='VERT':
                    wp_yaw = -math.radians(rack["pose"]["yaw"])
                else:
                    if wp[2]==wps[i+1][2]:
                        wp_yaw = math.atan2(wps[i+1][1]-wp[1],wps[i+1][0]-wp[0])
                    else:   
                        wp_yaw = wp_list[-1][3]

                wp_list.append([wp[0],wp[1],wp[2],wp_yaw])

            wp_list.append([wps[-1][0], wps[-1][1], wps[-1][2], wp_list[-1][3]])
            
            home_pos = [0,0,self.takeoff_alt]
            
            wp_list.append([home_pos[0],
                            home_pos[1],
                            home_pos[2],
                            math.atan2(home_pos[1]-wp_list[-1][1], home_pos[0]-wp_list[-1][0])])

        return wp_list


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
                rospy.loginfo('[AutoNavigator] WP[' + str(line_count) + '] >>  X: ' + str(wp_x) + ', Y: ' + str(wp_y)+ ', Z: ' + str(wp_z) + ' Yaw: '+ str(wp_yaw))
                prev_row = row  
            rospy.loginfo('[AutoNavigator] Got ' + str(line_count) + ' Waypoints from the file!')

            return wp_list
            # return True
    

    #TODO: Untested code below! Proceed with caution!!!
    # def generate_circle_wps(self, rad=5.0, start_x=1.0, start_y=0.0, dir=1):
    #     N = 50
    #     wp_list = []

    #     theta = np.linspace(0,dir*2*math.pi,N)
    #     print(theta)
    #     x = rad*np.cos(theta)
    #     y = rad*np.sin(theta)
    #     z = self.takeoff_alt+2.0*np.linspace(0,1,N)

    #     x += -x[0] + start_x
    #     y += -y[0] + start_y

    #     yaw_list = np.zeros(N)

    #     for i in range(N-1):
    #         yaw = math.atan2(y[i+1]-y[i], x[i+1]-x[i])
    #         yaw_list[i] = yaw

    #     yaw_list[N-1] = yaw_list[N-2]

    #     for i in range(N):
    #         wp_list.append([x[i], y[i], z[i], yaw_list[i]])
        
    #     print(wp_list)

    #     return wp_list
    
    # def generate_lem_wps(self, rad=5.0, start_x=1.0, start_y=0.0, dir=1):
    #     N = 150
    #     wp_list = []

    #     theta = np.linspace(0,dir*2*math.pi,N)
    #     print(theta)
    #     x = rad*np.cos(theta)/(1+pow(np.sin(theta),2))
    #     y = rad*np.sin(theta)*np.cos(theta)/(1+pow(np.sin(theta),2))
    #     z = self.takeoff_alt+2.0*np.linspace(0,1,N)

    #     x += -x[0] + start_x
    #     y += -y[0] + start_y

    #     yaw_list = np.zeros(N)

    #     for i in range(N-1):
    #         yaw = math.atan2(y[i+1]-y[i], x[i+1]-x[i])
    #         yaw_list[i] = yaw

    #     yaw_list[N-1] = yaw_list[N-2]

    #     for i in range(N):
    #         wp_list.append([x[i], y[i], z[i], yaw_list[i]])
        
    #     print(wp_list)

    #     return wp_list


if __name__ == '__main__':
    try:
        # Intialise the node
        rospy.init_node('waypoint_tracker', anonymous=False)
        # Instantiate the Node class
        nh = WPTrackerNode()
        # Do not terminate until user keyboard interrupt
        rospy.spin()
    except rospy.ROSInterruptException:
        pass