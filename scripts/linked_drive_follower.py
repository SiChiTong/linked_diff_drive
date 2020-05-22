#! /usr/bin/env python 

# Generate random path for diff drive
from __future__ import print_function
import numpy as np
import copy
import math

import rospy
import tf.transformations as t
from tf.broadcaster import TransformBroadcaster
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
#from std_msgs.msg import Header


class MarkerPub:
    def __init__(self, name, marker_type, rgba):
        ''' MARKERS '''
        self.name = name # str
        self.marker_type = marker_type # int
        if self.marker_type == 2: self.scale = [0.1, 0.1, 0.1] # SPHERE
        elif self.marker_type == 1: self.scale = [0.01, 0.01, 0.01] # CUBE
        elif self.marker_type == 0: self.scale = [0.5, 0.03, 0.03] # ARROW
        else: self.scale = [0.06, 0.06, 0.06] # OTHERS
        self.rgba = rgba # list
        self.array = MarkerArray() 
        ''' COUNTS '''
        self.COUNT = 0
        self.MARKER_MAX = 1
        ''' ROS node '''
        self.marker_pub = rospy.Publisher(self.name, MarkerArray, queue_size=10)

    def publish_marker(self, poses, quaternion=[0,0,0,1.0]):
        for pose in poses:

            m = Marker()
            m.header.frame_id = 'map'
            m.type = self.marker_type
            m.action = m.ADD
            m.scale.x = self.scale[0]
            m.scale.y = self.scale[1]
            m.scale.z = self.scale[2]
            m.color.r = self.rgba[0] 
            m.color.g = self.rgba[1] 
            m.color.b = self.rgba[2] 
            m.color.a = self.rgba[3]
            m.pose.orientation.x = quaternion[0] 
            m.pose.orientation.y = quaternion[1] 
            m.pose.orientation.z = quaternion[2] 
            m.pose.orientation.w = quaternion[3] 
            m.pose.position.x = pose[0] 
            m.pose.position.y = pose[1]
            m.pose.position.z = 0.0 
            
            ''' add new marker and remove the old one '''
            if len(poses) > self.MARKER_MAX: self.MARKER_MAX = len(poses)
            if self.COUNT > self.MARKER_MAX:
                self.array.markers.pop(0)

            self.array.markers.append(m)

            self.COUNT += 1 

        ''' Renumber the marker IDs '''
        id = 0
        for ma in self.array.markers:
            ma.id = id
            id += 1

        self.marker_pub.publish(self.array)

    def marker_transfer(self, x, y, child, parent="map"):

        time = rospy.Time.now()
        self.br.sendTransform( (x, y, 0), (0, 0, 0, 1.0), time, parent, child )
        

class LinkedDrive:

    def __init__(self, rot_vel_max=2.5718, # In rad/s
                       lin_vel_max=3.0, # In m/s
                       rot_acc=1.0, lin_acc=1.0): # in rad/s^2 and m/s^2

        ''' Shared params '''
        self.dt = 0.5 # sec
        self.L = 1.0 # dist between two solamr when connected with shelft

        ''' variables of FRONT car '''
        self.RATE = 10
        self.front_pose = [0.0, 0.0] # [x, y]
        self.front_ori = [0.0, 0.0, 0.0, 0.0] # [x, y, z, w]
        self.front_vel = [0.0, 0.0]  # [v_linear, w_angular]
        
        ''' variables and params of REAR car '''
        self.INIT_X = - self.L
        self.INIT_Y = 0.0
        self.cur_pose = [0.0, 0.0] # [x, y]
        self.cur_vel = [0.0, 0.0] # [lin, ang]
        self.cur_th = 0.0
        self.ROT_VEL = [-rot_vel_max, rot_vel_max]
        self.LIN_VEL = [-lin_vel_max, lin_vel_max]
        self.ROT_ACC = [-rot_acc, rot_acc]
        self.LIN_ACC = [-lin_acc, lin_acc]

        ''' ROS node '''
        self.f_sub_1 = rospy.Subscriber("/front_pose", PoseStamped, self.f_pose_update)
        self.f_sub_2 = rospy.Subscriber("/front_twist", Twist, self.f_twist_update)

    def f_pose_update(self, data):
        ''' update x, y coordinate '''
        self.front_pose[0] = data.pose.position.x
        self.front_pose[1] = data.pose.position.y
        
        ''' update quaternion '''
        self.front_ori[0] = data.pose.orientation.x
        self.front_ori[1] = data.pose.orientation.y
        self.front_ori[2] = data.pose.orientation.z
        self.front_ori[3] = data.pose.orientation.w

    def f_twist_update(self, data):
        self.front_vel[0] = data.linear.x
        self.front_vel[1] = data.angular.z


    def f_predict(self):
        ''' prediction of front car's trajectory using arc (combination of v and w) '''
        ''' front car state 0 '''
        (roll, pitch, f_th_0) = t.euler_from_quaternion(self.front_ori)
        f_x_0 = self.front_pose[0]
        f_y_0 = self.front_pose[1]
        ''' radius of the arc : r > 0, arc on the left; r < 0, arc on the right'''
        omega = self.front_vel[1]
        if omega == 0:
            r = 0
            d_x = self.front_vel[0] * self.dt * np.cos(f_th_0)
            d_y = self.front_vel[0] * self.dt * np.sin(f_th_0)
        else:
            r = self.front_vel[0] / omega
            ''' pose of end of arc before rotation '''
            d_x = r * np.sin(omega * self.dt) 
            d_y = r - r * np.cos(omega * self.dt) 
#        print("d_x, d_y in predict={0}".format([d_x, d_y]))
        ''' rotate for the theta '''
        PI = np.pi
        rot_mat = np.array([[np.cos(f_th_0), - np.sin(f_th_0)],
                            [np.sin(f_th_0), np.cos(f_th_0)]])
        [[f_x_1], [f_y_1]] = [[f_x_0], [f_y_0]] + np.dot(rot_mat, [[d_x], [d_y]])
        f_th_1 = f_th_0 + omega * self.dt 
#        print("\ncurrent : {0}".format([f_x_0, f_y_0]))
#        print("\ncurrent vel : {0}".format(self.front_vel))
#        print("\npredicted : {0}".format([f_x_1, f_y_1]))
        
        return [f_x_1, f_y_1, f_th_1]
        
    def get_potential_poses(self):
        ''' return potential locations (poses) for follower to be at '''
        res = 0.1 # rad, potential poses every __ rad
        [x, y] = self.f_predict()[:2]
        lst_rad = np.arange(0, 2 * np.pi, res)
        lst_poses = []
        for th in lst_rad:
            lst_poses.append([x + self.L * np.cos(th), y + self.L * np.sin(th)])
        return lst_poses

    def vels_from_pose(self, pose):

        self.cur_pose = self.front_pose # testing, should be deleted
        [row, pitch, self.cur_th] = t.euler_from_quaternion(self.front_ori)

        ''' return lin/ang velocities from given pose '''
        mat_rot_inv = np.array([ [np.cos(self.cur_th), np.sin(self.cur_th)],
                                 [-np.sin(self.cur_th), np.cos(self.cur_th)] ])
        [[dx], [dy]] = np.dot(mat_rot_inv, np.array([[pose[0]-self.cur_pose[0]],
                                                     [pose[1]-self.cur_pose[1]]]))

        if dy == 0: # only lin_vel, no rotation
            w = 0.0
            v = dx / self.dt
        else:
            r = (dx**2 + dy**2) / (2.0*dy)
            ''' w = omega and v = vel.x '''
            w = np.arcsin(dx /np.array(r)) / self.dt # 1x2 mat
            v = np.array(r) * np.array(w)
        
#        rospy.loginfo("v={0}; w={1}; ans={2}; dxdy={3}\n".format(v, w, self.front_vel, [dx,dy]))


    def check_link_dist(self):
        ''' checl the distance between front and follower '''
        pass
    
    def states_update(self):
        pass

    def follower_gen(self):
        pass


    def main(self):

        rospy.init_node('linked_drive')
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
             ''' Initialize markers '''
             front = MarkerPub(name='front_marker', marker_type=2, rgba=[1.0,1.0,1.0,1.0])
             front_predict = MarkerPub(name='front_predict_marker', marker_type=2, rgba=[0.0,1.0,0.0,0.3])
             predict_potential = MarkerPub(name='potential_poses', marker_type=1, rgba=[0.5,0.95,0.8,0.8])
             front_arrow = MarkerPub(name='front_arrow', marker_type=0, rgba=[1.0,1.0,1.0,1.0])
             front_predict_arrow = MarkerPub(name='front_predict_arrow', marker_type=0, rgba=[0.0,1.0,0.0,0.3])
             ''' get poses '''
             front_predict_pose = self.f_predict() #[x, y, th]
             fp_qua = t.quaternion_from_euler(0.0, 0.0, front_predict_pose[2])
             potential_poses = self.get_potential_poses()
             self.vels_from_pose(front_predict_pose[:2])

             ''' Publish markers (array) '''
             front.publish_marker([self.front_pose])
             front_arrow.publish_marker([self.front_pose], self.front_ori)
             front_predict.publish_marker([front_predict_pose])
             front_predict_arrow.publish_marker([front_predict_pose], fp_qua)
             predict_potential.publish_marker(potential_poses)
             #print(len(potential_poses))
            
             rate.sleep()

if __name__ == '__main__':

    try:
        test = LinkedDrive()
        test.main()

    except rospy.ROSInterruptException:
        pass
