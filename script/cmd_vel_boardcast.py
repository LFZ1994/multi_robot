#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from geometry_msgs.msg import Twist

def cmd_callback(data):
    for i in range(robot_num):     
        twist = Twist()
        twist.linear.x = data.linear.x
        twist.linear.y = data.linear.y
        twist.linear.z = data.linear.y
        twist.angular.x = data.angular.x
        twist.angular.y = data.angular.y
        twist.angular.z = data.angular.z
        twist.angular
        names['cmd_pub_%s'%i].publish(twist)  

if __name__ == '__main__': 
    try:
        rospy.init_node('cmd_vel_boardcast')
        robot_num = int(rospy.get_param('~robot_num','1'))
        if robot_num < 1:
            robot_num = 1
        twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
        rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
        info_string = "cmd_vel_boardcast running,robot_num:%d"%robot_num
        rospy.loginfo(info_string)
        names = locals()
        for i in range(robot_num):
            twist_cmd_topic_name = 'robot_%s%s'%(i,twist_cmd_topic)
            # names['cmd_pub_%s'%i]
            names['cmd_pub_%s'%i] = rospy.Publisher(twist_cmd_topic_name, Twist, queue_size=1)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.sh
        pass

