import numpy as np
import math
from tf import transformations as tf_utils

class GridPlanner():
    
    def __init__(self):
        self.grid_layouts = ['HOR','VERT']
        self.traversal_dir = 1
    
    # Class parameters setter methods
    def set_scan_area(self,width,height):
        self.rack_width = width
        self.rack_height = height
    
    def set_rack_depth(self, depth):
        self.rack_depth = depth
    
    def set_shelf_start_hgt(self, hgt):
        self.shelf_start_hgt = hgt
    
    def set_scan_margin(self, margin_hor, margin_vert):
        self.grid_margin_vert = margin_vert
        self.grid_margin_hor = margin_hor
    
    def set_scan_dist(self, scan_dist):
        self.scan_dist = scan_dist
    
    def set_grid_layout(self,layout_str):   
        if not layout_str in self.grid_layouts:
            print('Error! Incorrect grid layout provided; defaulting to HORIZONTAL layout')
            self.grid_layout = 'HOR'
        else:
            self.grid_layout = layout_str
    
    def set_grid_spacing(self,spacing):
        self.grid_spacing = spacing
    
    def set_num_pts_line(self, num):
        self.pts_count = num
    
    def set_rack_position(self, pos):
        self.pos = pos
    
    def set_rack_yaw(self, yaw):
        self.yaw = yaw

    def set_all_params(self, width, height, margin_hor, margin_vert, scan_dist, layout_str, spacing, num):
        self.set_scan_area(width, height)
        self.set_scan_margin(margin_hor, margin_vert)
        self.set_scan_dist(scan_dist)
        self.set_grid_layout(layout_str)
        self.set_grid_spacing(spacing)
        self.set_num_pts_line(num)

    # main method to generate the grid plan
    def generate_plan(self):
        waypoint_list = []

        curr_x = self.scan_dist+self.rack_depth

        # Horizontal mode
        if self.grid_layout=='HOR':
            self.num_levels = int(math.floor((self.rack_height-2*self.grid_margin_vert)/self.grid_spacing)+1)
            curr_z = self.grid_margin_vert
            
            if self.traversal_dir == 1:
                k = 1
                curr_y = self.grid_margin_hor
            elif self.traversal_dir == -1:
                k = -1
                curr_y = self.rack_width-self.grid_margin_hor
            
            point_spacing = (self.rack_width - 2*self.grid_margin_hor)/self.pts_count

            for i in range(self.num_levels):
                
                waypoint_list.append([curr_x, curr_y, curr_z])
                    
                for j in range(self.pts_count):
                    curr_y = curr_y + k*point_spacing
                    waypoint_list.append([curr_x, curr_y, curr_z])

                                
                k *= -1
                curr_z += self.grid_spacing

        # Vertical Mode
        else:
            self.num_levels = int(math.floor((self.rack_width-2*self.grid_margin_hor)/self.grid_spacing)+1)
            curr_y = self.grid_margin_hor
            
            if self.traversal_dir == 1:
                k = 1
                curr_z = self.grid_margin_vert
            elif self.traversal_dir == -1:
                k = -1
                curr_z = self.rack_height-self.grid_margin_vert
            
            point_spacing = (self.rack_height - 2*self.grid_margin_vert)/self.pts_count

            for i in range(self.num_levels):
                
                waypoint_list.append([curr_x, curr_y, curr_z])
                    
                for j in range(self.pts_count):
                    curr_z = curr_z + k*point_spacing
                    waypoint_list.append([curr_x, curr_y, curr_z])
                     
                k *= -1
                curr_y += self.grid_spacing
    
        self.numpy_wps = np.array(waypoint_list)
        self.tranform_wps_to_ff()
    
    # transform all the generated WPs to fixed frame
    def tranform_wps_to_ff(self):
        for i in range(len(self.numpy_wps)):
            self.numpy_wps[i,:] = np.matmul(self.numpy_wps[i,:],tf_utils.euler_matrix(0,0,-self.yaw,'rxyz')[:3,:3])
        self.numpy_wps += (self.pos+np.array([0,0,self.shelf_start_hgt])-self.numpy_wps[0,:])

    # Return the generated WPs
    def get_plan(self):
        return self.numpy_wps