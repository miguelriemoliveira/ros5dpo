#!/usr/bin/env python2

import rospy
import copy
#import re
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
#from math import sqrt
#from math import pow
from geometry_msgs.msg import Point
#from geometry_msgs.msg import PoseStamped
#from msl_msgs.msg import RobotStatus
#from msl_msgs.msg import RobotCommand
#import socket
#import sys
#import numpy as np
import math
import scipy
#from tf.transformations import euler_from_quaternion
#import roslib; roslib.load_manifest('msl_msgs')
#import rospy

 
## GLOBAL VARIABLES ##

def initROS():


    global g_agent_number
    print "agent_number=" + str(g_agent_number)

    global pub
 
def main():

    rospy.init_node('field_marker_publisher_node', anonymous=True)
    pub = rospy.Publisher('field_markers', MarkerArray, queue_size=10)

    field_lenght_x = rospy.get_param('/field/field_lenght_x')   
    field_lenght_y = rospy.get_param('/field/field_lenght_y')   
    line_width = rospy.get_param('/field/line_width')   
    circle_radius = rospy.get_param('/field/circle_radius')   
    penalty_area_lenght_x = rospy.get_param('/field/penalty_area_lenght_x')   
    penalty_area_lenght_y = rospy.get_param('/field/penalty_area_lenght_y')   
    goal_area_lenght_x = rospy.get_param('/field/goal_area_lenght_x')   
    goal_area_lenght_y = rospy.get_param('/field/goal_area_lenght_y')   

    ma = MarkerArray()

    #Green rectangle
    m = Marker()
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = "/world"
    m.ns = "field"
    m.id = 0
    m.type = 1

    m.pose.orientation.w = 1
    m.pose.position.z = -0.1
    m.scale.x = field_lenght_x + 1.5
    m.scale.y = field_lenght_y + 1.5
    m.scale.z = 0.005
    m.color.r = 0
    m.color.g = 0.3
    m.color.b = 0
    m.color.a = 1
    ma.markers.append(copy.deepcopy(m))

    #Field lines
    m.id += 1
    m.ns = "lines"
    m.type = 5 #line list
    m.pose.position.z = -0.05
    m.scale.x = line_width
    m.color.r = 1
    m.color.g = 1
    m.color.b = 1
    m.color.a = 1
    p = Point()

    p.x = -field_lenght_x/2
    p.y = -field_lenght_y/2
    m.points.append(copy.copy(p))
    p.x = field_lenght_x/2
    p.y = -field_lenght_y/2
    m.points.append(copy.copy(p))

    p.x = -field_lenght_x/2
    p.y = field_lenght_y/2
    m.points.append(copy.copy(p))
    p.x = field_lenght_x/2
    p.y = field_lenght_y/2
    m.points.append(copy.copy(p))

    p.x = -field_lenght_x/2
    p.y = 0
    m.points.append(copy.copy(p))
    p.x = field_lenght_x/2
    p.y = 0
    m.points.append(copy.copy(p))

    p.x = -field_lenght_x/2
    p.y = field_lenght_y/2
    m.points.append(copy.copy(p))
    p.x = -field_lenght_x/2
    p.y = -field_lenght_y/2
    m.points.append(copy.copy(p))

    p.x = field_lenght_x/2
    p.y = field_lenght_y/2
    m.points.append(copy.copy(p))
    p.x = field_lenght_x/2
    p.y = -field_lenght_y/2
    m.points.append(copy.copy(p))

    p.x = penalty_area_lenght_x/2
    p.y = field_lenght_y/2-penalty_area_lenght_y
    m.points.append(copy.copy(p))
    p.x = -penalty_area_lenght_x/2
    p.y = field_lenght_y/2-penalty_area_lenght_y
    m.points.append(copy.copy(p))

    p.x = penalty_area_lenght_x/2
    p.y = field_lenght_y/2-penalty_area_lenght_y
    m.points.append(copy.copy(p))
    p.x = penalty_area_lenght_x/2
    p.y = field_lenght_y/2
    m.points.append(copy.copy(p))

    p.x = -penalty_area_lenght_x/2
    p.y = field_lenght_y/2-penalty_area_lenght_y
    m.points.append(copy.copy(p))
    p.x = -penalty_area_lenght_x/2
    p.y = field_lenght_y/2
    m.points.append(copy.copy(p))

    p.x = penalty_area_lenght_x/2
    p.y = -field_lenght_y/2+penalty_area_lenght_y
    m.points.append(copy.copy(p))
    p.x = -penalty_area_lenght_x/2
    p.y = -field_lenght_y/2+penalty_area_lenght_y
    m.points.append(copy.copy(p))

    p.x = penalty_area_lenght_x/2
    p.y = -field_lenght_y/2+penalty_area_lenght_y
    m.points.append(copy.copy(p))
    p.x = penalty_area_lenght_x/2
    p.y = -field_lenght_y/2
    m.points.append(copy.copy(p))

    p.x = -penalty_area_lenght_x/2
    p.y = -field_lenght_y/2+penalty_area_lenght_y
    m.points.append(copy.copy(p))
    p.x = -penalty_area_lenght_x/2
    p.y = -field_lenght_y/2
    m.points.append(copy.copy(p))

    
    p.x = goal_area_lenght_x/2
    p.y = field_lenght_y/2-goal_area_lenght_y
    m.points.append(copy.copy(p))
    p.x = -goal_area_lenght_x/2
    p.y = field_lenght_y/2-goal_area_lenght_y
    m.points.append(copy.copy(p))

    p.x = goal_area_lenght_x/2
    p.y = field_lenght_y/2-goal_area_lenght_y
    m.points.append(copy.copy(p))
    p.x = goal_area_lenght_x/2
    p.y = field_lenght_y/2
    m.points.append(copy.copy(p))

    p.x = -goal_area_lenght_x/2
    p.y = field_lenght_y/2-goal_area_lenght_y
    m.points.append(copy.copy(p))
    p.x = -goal_area_lenght_x/2
    p.y = field_lenght_y/2
    m.points.append(copy.copy(p))

    p.x = goal_area_lenght_x/2
    p.y = -field_lenght_y/2+goal_area_lenght_y
    m.points.append(copy.copy(p))
    p.x = -goal_area_lenght_x/2
    p.y = -field_lenght_y/2+goal_area_lenght_y
    m.points.append(copy.copy(p))

    p.x = goal_area_lenght_x/2
    p.y = -field_lenght_y/2+goal_area_lenght_y
    m.points.append(copy.copy(p))
    p.x = goal_area_lenght_x/2
    p.y = -field_lenght_y/2
    m.points.append(copy.copy(p))

    p.x = -goal_area_lenght_x/2
    p.y = -field_lenght_y/2+goal_area_lenght_y
    m.points.append(copy.copy(p))
    p.x = -goal_area_lenght_x/2
    p.y = -field_lenght_y/2
    m.points.append(copy.copy(p))


    ma.markers.append(copy.deepcopy(m))


    #Draw circle
    m.id += 1 
    m.type = 4 #line list
    m.points = []
    m.pose.position.x = 0 
    m.pose.position.y = 0 
    m.scale.x = line_width
    num_facets = 100
    angle_step = math.pi*2./num_facets
    for (ind, angle) in enumerate(scipy.arange(0, math.pi*2.+angle_step, angle_step)):
        p = Point()
        p.x = math.cos(angle)*circle_radius
        p.y = math.sin(angle)*circle_radius
        m.points.append(p)

    ma.markers.append(copy.deepcopy(m))
      
    rate = rospy.Rate(1) 
    while not rospy.is_shutdown():

        print "in cycle"
        pub.publish(ma)
        
        ##rospy.loginfo(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
