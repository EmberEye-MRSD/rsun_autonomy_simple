import rospy
from sensor_msgs.msg import Range,LaserScan
from geometry_msgs.msg import PointStamped
import math
import numpy as np
# from tabulate import tabulate


# Class for the individual ROI sectors
class ROIScanSector:
    def __init__(self, id, fov, offset, prx_limit, glb_scan):
        # Global scan data
        self.glb_scan = glb_scan
        self.glb_ranges = None
        self.glb_num_pts = len(self.glb_scan.ranges)    
        self.glb_m = (self.glb_num_pts-1)/(self.glb_scan.angle_max-self.glb_scan.angle_min)
        self.glb_c = -(self.glb_scan.angle_min)*(self.glb_num_pts-1)/(self.glb_scan.angle_max-self.glb_scan.angle_min)
        
        # Local ROI params
        self.id = id
        self.prx_limit = prx_limit
        self.range_msg = Range()
        self.scan_msg = LaserScan()
        self.fov = fov #angular spread of the sector
        self.offset = offset #offset from the FWD direction of the MAV, ACW is +ve
        # eval the sector start and end angles
        self.start_angle = offset-fov/2
        self.end_angle = offset+fov/2
        self.flag_collision = False
        self.sect_idxs = []
        self.obs_pos = []

        # wrap the angles to confine the values with the angular spread of the global scan
        if abs((self.glb_scan.angle_max-self.glb_scan.angle_min)-2*math.pi) < 0.10:
            self.start_angle = self.wrap_angle(self.start_angle)
            self.end_angle = self.wrap_angle(self.end_angle)
        else:                                                                                                                            
            self.constrain_sector()

        # flag used to ignore sector if its invalid
        self.flag_data_invalid = True
        self.ranges = []
        self.range_min = np.nan
        self.range_max = np.nan
        self.range_avg = np.nan

        rospy.logwarn("[CollisionPreventionInterface] Creating ROI:\n"
            +"__________________________________________________"
            +"\nID:          "+str(self.id)
            +"\nFOV:         "+str(math.degrees(self.fov))
            +"\nOffset:      "+str(math.degrees(self.offset))
            +"\nPRX limit:   "+str(self.prx_limit)
            +"\nStart angle: "+str(math.degrees(self.start_angle))
            +"\nEnd angle:   "+str(math.degrees(self.end_angle))
            +"\n__________________________________________________"
            ) 
    
        self.range_pub = rospy.Publisher(('/collision_interface/sectors/'+str(self.id)+"/range_stats"), Range, queue_size=1)
        self.scan_pub = rospy.Publisher(('/collision_interface/sectors/'+str(self.id)+"/scan"), LaserScan, queue_size=1)
        self.obs_pos_pub = rospy.Publisher(('/collision_interface/sectors/'+str(self.id)+"/obstacle_pos"), PointStamped, queue_size=1)



    # Method to update the global range data
    def update_glb_ranges(self, ranges):
        self.glb_ranges = np.array(ranges)
    
    # Method to process the ranges in the ROI
    def process_sector(self):
        self.flag_collision = False
        # Find idxs of points within the angular spread
        self.sect_idxs = [self.ang_to_idx(self.wrap_angle(x)) for x in 
                np.arange(self.start_angle,self.start_angle+self.fov,
                self.glb_scan.angle_increment)]
        
        # extract pts corresponding to above idxs
        raw_sect_ranges = self.glb_ranges[self.sect_idxs]
        
        self.ranges = []
        # remove invalid ranges
        for d in raw_sect_ranges:
            if d < self.glb_scan.range_max and d > self.glb_scan.range_min:
                self.ranges.append(d)
            else:
                self.ranges.append(np.nan)
        
        self.ranges = np.array(self.ranges)

        # If pts list not empty, eval the range metrics
        if(not self.ranges.size==0):
            self.flag_data_invalid = False
            self.range_min = np.nanmin(self.ranges)
            self.range_max = np.nanmean(self.ranges)
            self.range_avg = np.nanmax(self.ranges)
            self.flag_collision = (self.prx_limit>self.range_min)
        
        if self.range_min == np.nan:
            self.flag_collision = False
    
    # return collision status
    def get_collision_status(self):
        return self.flag_collision

    # convenience method for idx based angle lookup
    def idx_to_ang(self, idx):
        return (idx-self.glb_c)/self.glb_m

    # convenience method for angle based range lookup
    def ang_to_idx(self,theta):
        # print(theta)
        # Use y = mx +c form for linear fitting
        return int(self.glb_m*theta+self.glb_c)

    #  wrap angle wrt global range angular limits
    def wrap_angle(self, ang):
        x = math.fmod(ang-self.glb_scan.angle_max,2*math.pi)
        if (x < 0):
            x += 2*math.pi
        return x+self.glb_scan.angle_min

    # constrain the sector limits to the global scan fov limits
    def constrain_sector(self):
        self.start_angle = max(self.start_angle, self.glb_scan.angle_min)
        self.end_angle = min(self.end_angle, self.glb_scan.angle_max)

    # method to fill LaserScan msg for debugging 
    def send_scan_msg(self):
        self.scan_msg.header.frame_id = self.glb_scan.header.frame_id
        self.scan_msg.header.stamp = rospy.Time.now()
        self.scan_msg.angle_min = self.offset-self.fov/2
        self.scan_msg.angle_max = self.offset+self.fov/2
        self.scan_msg.range_max = self.glb_scan.range_max
        self.scan_msg.range_min = self.glb_scan.range_min
        self.scan_msg.angle_increment = self.glb_scan.angle_increment
        # self.scan_msg.ranges = self.glb_ranges[[self.ang_to_idx(self.wrap_angle(x)) for x in
        #                                         np.arange(self.start_angle,self.start_angle+self.fov,
        #                                         self.glb_scan.angle_increment)]]
        self.scan_msg.ranges = self.ranges

        self.scan_pub.publish(self.scan_msg)

    # method to fill the Range msg for range stats
    def send_range_msg(self):
        self.range_msg.radiation_type = self.range_msg.INFRARED
        self.range_msg.header.stamp = rospy.Time.now()
        self.range_msg.header.frame_id='base_link'
        self.range_msg.range = self.range_avg
        self.range_msg.min_range = self.range_min
        self.range_msg.max_range = self.range_max
        self.range_msg.field_of_view = self.fov

        self.range_pub.publish(self.range_msg)
    
    def get_sect_idxs(self):
        return self.sect_idxs

    # get obstacle position wrt the MAV body FLU frame 
    def get_obstacle_pos(self):
        self.obs_pos = np.array([np.nan,np.nan])
        obs_pos_raw = []

        for i in range(self.glb_num_pts):
            if i in self.sect_idxs and abs(self.glb_ranges[i]-self.prx_limit)<1.50:
                    # obs_angs.append(self.idx_to_ang(i))
                    # obs_rngs.append(self.glb_ranges[i])
                    obs_r = self.glb_ranges[i]
                    obs_theta = self.idx_to_ang(i)
                    obs_pos_raw.append([obs_r*math.cos(obs_theta), obs_r*math.sin(obs_theta)])
        
        obs_pos_raw = np.array(obs_pos_raw)
        
        obs_msg = PointStamped()
        obs_msg.header.stamp = rospy.Time.now()
        obs_msg.header.frame_id = "base_link"
        obs_msg.point.x = np.nan
        obs_msg.point.y = np.nan
        obs_msg.point.z = 0.0
        
        if not len(obs_pos_raw)==0:
            self.obs_pos = np.array([obs_pos_raw[:,0].mean(),obs_pos_raw[:,1].mean()])
            # print("X: "+str(self.obs_pos[0]))
            # print("Y: "+str(self.obs_pos[1]))

        obs_msg.point.x = self.obs_pos[0]
        obs_msg.point.y = self.obs_pos[1]
        self.obs_pos_pub.publish(obs_msg)
 
        return self.obs_pos
