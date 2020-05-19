#! /usr/bin/env python 

# Generate random path for diff drive
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


class LinkedDrive:

    def __init__(self, rot_vel_max=2.5718, # In rad/s
                       lin_vel_max=3.0, # In m/s
                       rot_acc=1.0, lin_acc=1.0): # in rad/s^2 and m/s^2

        #self.br = TransformBroadcaster()
        self.marker_array = MarkerArray()
        self.f_array = MarkerArray() # front marker
        self.fp_array = MarkerArray() # front-predict marker
        self.MARKER_MAX = 20
        self.f_COUNT = 0
        self.fp_COUNT = 0

        ''' Shared params '''
        self.dt = 1 # sec

        ''' variables of FRONT car '''
        self.RATE = 10
        self.front_pose = [0.0, 0.0] # [x, y]
        self.front_ori = [0.0, 0.0, 0.0, 0.0] # [x, y, z, w]
        self.front_vel = [0.0, 0.0]  # [v_linear, w_angular]
        
        ''' variables and params of REAR car '''
        self.INIT_X = 0.0
        self.INIT_Y = 0.0
        self.cur_rot_vel = 0.0
        self.cur_lin_vel = 0.0
        self.cur_th = 0.0
        self.ROT_VEL = [-rot_vel_max, rot_vel_max]
        self.LIN_VEL = [-lin_vel_max, lin_vel_max]
        self.ROT_ACC = [-rot_acc, rot_acc]
        self.LIN_ACC = [-lin_acc, lin_acc]

        ''' ROS node '''
        rospy.init_node('linked_drive')
        topic = 'visualization_marker_array'
        self.marker_pub = rospy.Publisher(topic, MarkerArray, queue_size=10)
        self.f_sub = rospy.Subscriber("/front_pose", PoseStamped, self.f_pose_update)
        self.f_sub = rospy.Subscriber("/front_twist", Twist, self.f_twist_update)

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

    def marker_gen(self, x, y, array, count, r=1.0, g=1.0, b=0.0):

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x 
        marker.pose.position.y = y 
        marker.pose.position.z = 0.0 
        
        array = self.marker_array
        ''' add new marker and remove the old one '''
        if count > self.MARKER_MAX:
            array.markers.pop(0)
        array.markers.append(marker)
        count += 1 # this didn't work???
        #print("\n{0}".format(count))

        ''' Renumber the marker IDs '''
        id = 0
        for m in array.markers:
            m.id = id
            id += 1

    def marker_transfer(self, x, y, child, parent="map"):

        time = rospy.Time.now()
        self.br.sendTransform( (x, y, 0), (0, 0, 0, 1.0), time, parent, child )

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
        else:
            r = self.front_vel[0] / omega
        ''' pose of end of arc before rotation '''
        d_x = r * np.sin(omega * self.dt) 
        d_y = r - r * np.cos(omega * self.dt) 
        ''' rotate for the theta '''
        PI = np.pi
        rot_mat = np.array([[np.cos(f_th_0), - np.sin(f_th_0)],
                            [np.sin(f_th_0), np.cos(f_th_0)]])
        [[f_x_1], [f_y_1]] = [[f_x_0], [f_y_0]] + np.dot(rot_mat, [[d_x], [d_y]])
        f_th_1 = f_th_0 + omega * self.dt 
        print("\ncurrent : {0}".format([f_x_0, f_y_0]))
        print("\ncurrent vel : {0}".format(self.front_vel))
        print("\npredicted : {0}".format([f_x_1, f_y_1]))
        
        return [f_x_1, f_y_1, f_th_1]
        

    
    def states_update(self):
        pass

    def follower_gen(self):
        pass


    def main(self):

        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            ''' update the front xy coord '''
            [x_cur, y_cur] = self.front_pose
            self.marker_gen(x=x_cur, y=y_cur, array=self.f_array, count=self.f_COUNT, r=0.6, g=0.75, b=0.97)
            #self.marker_transfer(x_cur, y_cur, child="front")
            ''' update the predicted front xy coord '''
            [x_p, y_p, th_p] = self.f_predict()
            self.marker_gen(x=x_p, y=y_p, array=self.fp_array, count=self.fp_COUNT, r=1.0, g=1.0, b=1.0)
            #self.marker_transfer(x_p, y_p, child="front_predict")
            ''' Publish the MarkerArray '''
            self.marker_pub.publish(self.marker_array)
            self.f_COUNT += 1
            self.fp_COUNT += 1

            rate.sleep()

if __name__ == '__main__':

    try:
        test = LinkedDrive()
        test.main()

    except rospy.ROSInterruptException:
        pass
