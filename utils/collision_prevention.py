#!/usr/bin/python3

import rospy
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool
import math
import numpy as np
from copy import deepcopy
from prx_scan_sector import ROIScanSector


class CollisionPreventionInterface:

    def __init__(self):
        
        rospy.loginfo("[CollisionPreventionInterface] Collision Prevention interface loaded!")

        # Initialise timeout counts
        self.last_scan_time = rospy.Time.now()
        self.time_last_collision = rospy.Time.now()

        # Initialise flags
        self.flag_interface_ready = False
        self.flag_en_avoidance = False
        self.flag_en_wall_follow = False
        self.flag_collision = False
        self.flag_recv_scan = False
        self.flag_scan_timeout = False
        self.flag_invalid_cfg = True
        self.evade_vec = np.zeros(2)

        rospy.Timer(rospy.Duration(0.50), self.avoidance_en_checker)

    
    def init_interface(self):
        self.flag_interface_ready = True

        if not rospy.has_param('~avoid_sectors'):
            rospy.logerr("[CollisionPreventionInterface] Critical! no ROIs provided.")
            self.flag_invalid_cfg = True
        else:
            # user-defined ROIs for CP processing
            self.sect_info_list = rospy.get_param('~avoid_sectors')
            self.k_evade = rospy.get_param('~dyn_avoid_gain', 1.50)
            self.k_wall_follow = rospy.get_param('~wall_follow_gain', 2.50)
            self.flag_invalid_cfg = False
        
        if not self.flag_invalid_cfg:
            self.sect_obj_list = []
            self.obstacle_list = [] # contains the position of the obstacles relative to the MAV frame
            self.scan_msg = LaserScan()
            self.data_timeout_secs = rospy.get_param('~scan_data_timeout', 5.0) # data timeout period
            self.dt_collide_reset = rospy.Duration(rospy.get_param('~collision_reset_timeout', 3.0)) # collision flag reset period

            #TODO Sensor data checker loop
            # rospy.Timer(rospy.Duration(0.10), self.timer_cb, reset=True)

            # Subscribers
            self.status_pub = rospy.Publisher('/collision_interface/status', Bool, queue_size=0)
            rospy.Subscriber('/laser/scan', LaserScan, self.scan_cb)

        else:
            rospy.logwarn("[CollisionPreventionInterface] Avoidance disabled, invalid config!")
            
        
    def scan_cb(self, data):

        if not self.flag_en_avoidance:
            return

        self.last_scan_time = rospy.Time.now()
        self.scan_msg = data
        # Reset timeout flag
        if self.flag_scan_timeout:
            self.flag_scan_timeout = False

        if not self.flag_recv_scan:
            rospy.loginfo("[CollisionPreventionInterface] Received scan data!")

            self.flag_recv_scan = True
        
            for info in self.sect_info_list:
                self.sect_obj_list.append(ROIScanSector(info["id"],info["fov"],info["offset"],info["prx_limit"],data))
            
        else:
            for sect in self.sect_obj_list:
                sect.update_glb_ranges(data.ranges)
                sect.process_sector()

            self.eval_collision()
    
    # main scan processing method
    def eval_collision(self):
        curr_col_flag = False
        obs_pos_list = []
        evade_vec = np.zeros(2)
        # use all available sectors for collision evaluation
        for sect in self.sect_obj_list:
            # Ignore sectors with invalid data
            if not(sect.flag_data_invalid):
                sect.send_range_msg()
                sect.send_scan_msg()

                # evaluate the evasion command from each sector
                obs_pos = sect.get_obstacle_pos()

                if not np.isnan(np.linalg.norm(obs_pos)):
                    obs_dist = np.linalg.norm(obs_pos)

                    # if not self.flag_en_wall_follow:
                    if obs_dist < sect.prx_limit:
                    # Add local evasion cmd with global evasion cmd vector
                        evade_vec+=-(self.k_evade*(sect.prx_limit*obs_pos/obs_dist)-obs_pos)
                    # else:
                    #     if abs(obs_pos[0]) < sect.prx_limit:
                    #         if obs_pos[0] > 0:
                    #             evade_vec+=np.array([self.k_wall_follow*(obs_pos[0]-sect.prx_limit), 0.0])
                    #         else:
                    #             evade_vec+=-np.array([self.k_wall_follow*(obs_pos[0]-sect.prx_limit), 0.0])
                                
                    # else:
                    #     print("Error: "+str(obs_dist-sect.prx_limit))
                    #     evade_vec+=self.k_wall_follow*(obs_dist-sect.prx_limit)

                self.evade_vec = evade_vec
                obs_pos_list.append(sect.get_obstacle_pos())
                if sect.get_collision_status():
                    curr_col_flag = True
                     
                    
        time_now = rospy.Time.now()
        
        # Add collision flag time persistence to prevent oscillations
        if (self.flag_collision and not curr_col_flag and
            time_now-self.time_last_collision < self.dt_collide_reset):
            self.flag_collision = True
        elif not self.flag_collision and curr_col_flag:
            self.time_last_collision = rospy.Time.now()
            self.flag_collision = True
            rospy.logwarn_throttle(5.0, "[CollisionPreventionInterface] Obstacle proximity warning!")
        elif not self.flag_collision and not curr_col_flag:
            self.flag_collision = False
        elif self.flag_collision and curr_col_flag:
            self.flag_collision = True
            self.time_last_collision = rospy.Time.now()
        else:
            self.flag_collision = False
            rospy.loginfo_throttle(5.0, "[CollisionPreventionInterface] Obstacle proximity alarm cleared!")
        
        # Publish collision check status
        self.status_pub.publish(Bool(self.flag_collision))
        
        # reset data validity flag
        for sect in self.sect_obj_list:
            sect.flag_data_invalid = True

    # async callback to check for data loss timeout
    def timer_cb(self, event):
        if rospy.Time.now() - self.last_scan_time > rospy.Duration(self.data_timeout_secs):
            self.flag_scan_timeout = True
            rospy.logerr("[CollisionPreventionInterface] Sensor data timed-out!")
    
    # return collision evaluation result
    def get_collision_status(self):
        return deepcopy(self.flag_collision)
    
    # async callback to start the interface on request
    def avoidance_en_checker(self, event):
        if not self.flag_en_avoidance:
            rospy.logwarn_once("[CollisionPreventionInterface] Avoidance module in standby...")
        else:
            if not self.flag_interface_ready:
                self.init_interface()
    
    # enable/disable collision interface
    def toggle_avoidance(self, flag):
        self.flag_en_avoidance = flag
        if flag:
            rospy.loginfo("[CollisionPreventionInterface] Avoidance activated by user!")
        else:
            rospy.logwarn("[CollisionPreventionInterface] Avoidance disabled by user!")
    
    # return evasion command
    def get_evade_cmd(self):
        return self.evade_vec
