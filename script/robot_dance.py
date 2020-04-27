#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 wechange tech.
# Developer: FuZhi Liu 
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
import tf
from enum import Enum
import math
import threading

class single_dance:
    def __init__(self,data):
        self.deviceok = False
        self.current_theta = 0.0
        self.last_current_theta = 0.0 
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        self.movement_start = False
        self.desired_dist = 0.0
        self.desired_theta = 0.0
        self.lastError = 0.0
        self.start_pos_x = 0.0
        self.start_pos_y = 0.0
        self.StateOfDance = Enum('StateOfDance', 'idle Tunning Straighting dance_done')
        self.current_stage_of_dance = self.StateOfDance.idle.value
        twist_cmd_topic = '/cmd_vel'
        odom_cmd_topic = '/odom'
        self.twist_cmd_topic_name = 'robot_%s%s'%(data,twist_cmd_topic)
        self.odom_cmd_topic_name = 'robot_%s%s'%(data,odom_cmd_topic)
        self.cmd_pub = rospy.Publisher(self.twist_cmd_topic_name, Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_cmd_topic_name, Odometry, self.odom_callback, queue_size=1)

    def thread(self):
        loop_rate = rospy.Rate(100)
        # self.MovementStraight(0.2)
        while not rospy.is_shutdown():
            if self.current_stage_of_dance != self.StateOfDance.idle.value:
                self.Dancing()
            # else:
            #     print self.current_stage_of_dance
            loop_rate.sleep()

    def odom_callback(self,odom_msg):
        self.deviceok = True
        quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        # self.current_theta = self.euler_from_quaternion(quaternion)
        self.current_theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta
        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y

        # info_string = "%s:%df %f %f"%(self.odom_cmd_topic_name,self.current_pos_x,self.current_pos_y,self.current_theta)
        # rospy.logdebug(info_string)

    def MovementTurn(self,desired_theta):
        self.desired_theta = desired_theta
        err_theta = self.current_theta - self.desired_theta  
        self.current_stage_of_dance = self.StateOfDance.Tunning.value    
        Kp = 0.6
        Kd = 0.1
        angular_z = Kp * err_theta + Kd * (err_theta - self.lastError)
        if angular_z != 0:
            if abs(angular_z) < 0.1:
                angular_z = 0.1*abs(angular_z)/angular_z
        self.lastError = err_theta
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -angular_z
        self.cmd_pub.publish(twist)

        return err_theta
    def MovementStraight(self, desired_dist):
        # self.movement_start = True
        self.desired_dist = desired_dist
        self.current_stage_of_dance = self.StateOfDance.Straighting.value
        if self.desired_dist > 0.001:
            err_pos = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - self.desired_dist
        else:
            err_pos = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) + self.desired_dist
        # rospy.loginfo("Parking_Straight")
        twist = Twist()
        if abs(err_pos) > 0.005:
            if self.desired_dist > 0.0001:
                twist.linear.x = -0.06*abs(err_pos)/err_pos
            else:
                twist.linear.x = 0.06*abs(err_pos)/err_pos
        else:
           twist.linear.x = 0.0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.cmd_pub.publish(twist)
        return err_pos

    def Dancing(self):
        while not rospy.is_shutdown():
            # rospy.loginfo("Dancing Running")
            if self.current_stage_of_dance == self.StateOfDance.Straighting.value:
                if self.movement_start == False:
                    # rospy.loginfo("MovementStraight")
                    self.lastError = 0.0
                    self.start_pos_x = self.current_pos_x
                    self.start_pos_y = self.current_pos_y
                    self.movement_start = True

                error = self.MovementStraight(self.desired_dist)
                if math.fabs(error) < 0.005:
                    self.current_stage_of_dance = self.StateOfDance.idle.value
                    # rospy.loginfo("Straighting finished")
                    twist = Twist()
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.x = 0
                    twist.angular.y = 0
                    twist.angular.z = 0
                    self.cmd_pub.publish(twist)
                    rospy.sleep(1)
                    self.movement_start = False 
                else:
                    print error
            if self.current_stage_of_dance == self.StateOfDance.Tunning.value:
                if self.movement_start == False:
                    # rospy.loginfo("MovementTurn")
                    self.lastError = 0.0
                    self.start_pos_x = self.current_pos_x
                    self.start_pos_y = self.current_pos_y
                    self.movement_start = True

                error = self.MovementTurn(self.desired_theta)
                if math.fabs(error) < 0.02:
                    self.current_stage_of_dance = self.StateOfDance.idle.value
                    # rospy.loginfo("Tunning finished")
                    twist = Twist()
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.x = 0
                    twist.angular.y = 0
                    twist.angular.z = 0
                    self.cmd_pub.publish(twist)
                    rospy.sleep(1)
                    self.movement_start = False  

class dance:
    def __init__(self):
        self.robot_num = int(rospy.get_param('~robot_num','2'))
        self.MovementDone = True
        self.names = names = locals()
        for i in range(self.robot_num):
            names['Dancer_%s'%i] = single_dance(i)
            threading.Thread(target=self.names['Dancer_%s'%i].thread, args=()).start()
        self.dance()

    def dance(self):
        #turn to 0 degree
        while True:
            for i in range(self.robot_num):
                if self.MovementDone == True:
                    self.names['Dancer_%s'%i].MovementTurn(0)
            self.MovementDone = True
            rospy.sleep(0.1)
            for i in range(self.robot_num):    
                if  self.names['Dancer_%s'%i].current_stage_of_dance == self.names['Dancer_%s'%i].StateOfDance.idle.value:
                    if self.MovementDone == True:
                        self.MovementDone = True
                else:
                    self.MovementDone = False
            if self.MovementDone:
                break
        # move foreword 0.5m
        while True:
            for i in range(self.robot_num):
                if self.MovementDone == True:
                    self.names['Dancer_%s'%i].MovementStraight(0.5)
            self.MovementDone = True
            rospy.sleep(0.1)
            for i in range(self.robot_num):    
                if  self.names['Dancer_%s'%i].current_stage_of_dance == self.names['Dancer_%s'%i].StateOfDance.idle.value:
                    if self.MovementDone == True:
                        self.MovementDone = True
                else:
                    self.MovementDone = False
            if self.MovementDone:
                break
        rospy.sleep(1)
        # turn to 180 degree
        while True:
            for i in range(self.robot_num):
                if self.MovementDone == True:
                    self.names['Dancer_%s'%i].MovementTurn(3.1415)
            self.MovementDone = True
            rospy.sleep(0.1)
            for i in range(self.robot_num):    
                if  self.names['Dancer_%s'%i].current_stage_of_dance == self.names['Dancer_%s'%i].StateOfDance.idle.value:
                    if self.MovementDone == True:
                        self.MovementDone = True
                else:
                    self.MovementDone = False
            if self.MovementDone:
                break
        rospy.sleep(1)
        # move backword 0.5m
        while True:
            for i in range(self.robot_num):
                if self.MovementDone == True:
                    self.names['Dancer_%s'%i].MovementStraight(-0.5)
            self.MovementDone = True
            rospy.sleep(0.1)
            for i in range(self.robot_num):    
                if  self.names['Dancer_%s'%i].current_stage_of_dance == self.names['Dancer_%s'%i].StateOfDance.idle.value:
                    if self.MovementDone == True:
                        self.MovementDone = True
                else:
                    self.MovementDone = False
            if self.MovementDone:
                break
        rospy.sleep(1)
        # turn to 180 degree
        while True:
            for i in range(self.robot_num):
                if self.MovementDone == True:
                    self.names['Dancer_%s'%i].MovementTurn(3.1415)
            self.MovementDone = True
            rospy.sleep(0.1)
            for i in range(self.robot_num):    
                if  self.names['Dancer_%s'%i].current_stage_of_dance == self.names['Dancer_%s'%i].StateOfDance.idle.value:
                    if self.MovementDone == True:
                        self.MovementDone = True
                else:
                    self.MovementDone = False
            if self.MovementDone:
                break
        rospy.sleep(1)
        # move forword 1.0m
        while True:
            for i in range(self.robot_num):
                if self.MovementDone == True:
                    self.names['Dancer_%s'%i].MovementStraight(1.0)
            self.MovementDone = True
            rospy.sleep(0.1)
            for i in range(self.robot_num):    
                if  self.names['Dancer_%s'%i].current_stage_of_dance == self.names['Dancer_%s'%i].StateOfDance.idle.value:
                    if self.MovementDone == True:
                        self.MovementDone = True
                else:
                    self.MovementDone = False
            if self.MovementDone:
                break
        rospy.sleep(1)
        #turn to 0 degree
        while True:
            for i in range(self.robot_num):
                if self.MovementDone == True:
                    self.names['Dancer_%s'%i].MovementTurn(0)
            self.MovementDone = True
            rospy.sleep(0.1)
            for i in range(self.robot_num):    
                if  self.names['Dancer_%s'%i].current_stage_of_dance == self.names['Dancer_%s'%i].StateOfDance.idle.value:
                    if self.MovementDone == True:
                        self.MovementDone = True
                else:
                    self.MovementDone = False
            if self.MovementDone:
                break
        rospy.loginfo("dance is started finished")
        while not rospy.is_shutdown():
            pass
        # rospy.signal_shutdown("dance is started finished!")
if __name__ == '__main__':
    try:
        # 初始化ros节点
        dancenode = rospy.init_node("multi_robot_dance")
        rospy.loginfo("dance is started")
        # while not rospy.is_shutdown():
        dance()
        rospy.spin()

    except KeyboardInterrupt:
        print "Shutting down dance node."