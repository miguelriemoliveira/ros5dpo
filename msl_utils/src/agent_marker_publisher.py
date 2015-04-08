#!/usr/bin/env python

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
    pub = rospy.Publisher('agent_markers', MarkerArray, queue_size=10)

    agent_number = rospy.get_param('~agent_number')   
    print "agent_number = " + str(agent_number)

    height = rospy.get_param('/agent/height')   
    print "height = " + str(height)

    radius = rospy.get_param('/agent/radius')   
    print "radius = " + str(radius)


    ma = MarkerArray()

    #Green rectangle
    m = Marker()
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = "base_footprint_agent" + str(agent_number)
    m.ns = "agent"
    m.id = 0
    m.type = 3
    m.frame_locked = True

    m.pose.orientation.w = 1
    m.pose.position.z = 0
    m.scale.x = radius
    m.scale.y = radius
    m.scale.z = height
    m.color.r = 0.2
    m.color.g = 0.2
    m.color.b = 0.2
    m.color.a = 1
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
