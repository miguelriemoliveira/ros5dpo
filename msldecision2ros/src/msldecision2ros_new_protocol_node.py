#!/usr/bin/env python2
# license removed for brevity
import rospy
import re
from math import sqrt
from math import pow
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from msl_msgs.msg import RobotStatus
from msl_msgs.msg import RobotCommand
import socket
import sys
import numpy as np
import math
from tf.transformations import euler_from_quaternion
import roslib; roslib.load_manifest('msl_msgs')
import rospy

 
## GLOBAL VARIABLES ##

g_local_ip_address = '127.0.0.1'   # Symbolic name meaning all available interfaces
#g_remote_ip_address = '172.16.33.100'   # Symbolic name meaning all available interfaces
g_remote_ip_address = '127.0.0.1'   # Symbolic name meaning all available interfaces
g_receive_port = 5002 # Arbitrary non-privileged port
g_send_port = 5003 # Arbitrary non-privileged port
s = socket.socket()
s_receive = socket.socket()
pub = []
g_max_ball_distance = 2
g_agent_number = 1


def ReceiveUDP():
    data, addr = s_receive.recvfrom(1024)


    return data
 
def robotStatusCallback(data):
    print "I am in a robotStatusCallback"

    ## FOR THE AGENT
    msg = []
    msg.append(round(-data.agent_pose.pose.position.y,3))
    msg.append(round(data.agent_pose.pose.position.x,3))

    #quaternion q
    q =  np.array([data.agent_pose.pose.orientation.x, data.agent_pose.pose.orientation.y, data.agent_pose.pose.orientation.z, data.agent_pose.pose.orientation.w])
    (roll,pitch,yaw) = euler_from_quaternion(q);
    msg.append(round(yaw+3.14/2,3))


    ## FOR THE BALL
    msg.append(round(-data.ball_pose.pose.position.y,3))
    msg.append(round(data.ball_pose.pose.position.x,3))

    dist = math.sqrt(math.pow(data.agent_pose.pose.position.x - round(data.ball_pose.pose.position.x,3),2) +
            math.pow(data.agent_pose.pose.position.y - round(data.ball_pose.pose.position.y,3),2))

    if dist < g_max_ball_distance:
        msg.append(1)
    else:
        msg.append(0)
    
    if data.has_ball == 1:
        msg.append(1)
    else:
        msg.append(0)


    #Remove the first and last elements []
    msg_str = str(msg)
    if len(msg_str)>3:
        msg_str = msg_str[1:len(msg_str)-1]
    s.sendto(msg_str , (g_remote_ip_address,g_send_port))

    ##
    # @brief Configures UDP sockets
    #
    # @return 
def initUDP():
    
    #Configure sending socket
    try :
        global s
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print 'Socket created'
    except socket.error, msg :
        print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        sys.exit()
     
    #Configure receiving socket
    try :
        global s_receive
        s_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print 'Socket created'
    except socket.error, msg :
        print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        sys.exit()
     
    #Bind socket to local host and port
    try:
        s_receive.bind(('', g_receive_port))
    except socket.error , msg:
        print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]

    print 'Socket bind complete. Listen port=' + str(g_receive_port) + " send port=" + str(g_send_port)

def initROS():

    rospy.init_node('msldecision2ros_node', anonymous=True)

    global g_agent_number
    g_agent_number = rospy.get_param('~agent_number')   
    print "agent_number=" + str(g_agent_number)

    global g_max_ball_distance
    g_max_ball_distance = rospy.get_param('~max_ball_distance')   
    print "max_ball_distance=" + str(g_max_ball_distance)

    global g_local_ip_address
    g_local_ip_address = rospy.get_param('~local_ip_address')   
    print "local_ip_address=" + str(g_local_ip_address)

    global remote_ip_address
    g_remote_ip_address = rospy.get_param('~remote_ip_address')   
    print "remote_ip_address=" + str(g_remote_ip_address)

    global receive_port
    g_receive_port = rospy.get_param('~receive_port')   
    print "receive_port=" + str(g_receive_port)

    global send_port
    g_send_port = rospy.get_param('~send_port')   
    print "send_port=" + str(g_send_port)

    global pub
    pub = rospy.Publisher('robot_command', RobotCommand, queue_size=10)

    rospy.Subscriber("robot_status", RobotStatus, robotStatusCallback)

def processUDPMessage(m,rc):

    l = re.split(',', m)
    print l

    if not len(l) == 8:
        return False

    rc.header.frame_id = "agent_" + str(g_agent_number)
    rc.header.stamp = rospy.Time.now()
    rc.velocity.linear.x = float(l[0])
    rc.velocity.linear.y = float(l[1])
    rc.velocity.angular.z = float(l[2])
    rc.kick_power = np.uint8(l[3])
    rc.high_kick = np.bool(np.uint8(l[4]))
    rc.low_kick = np.bool(np.uint8(l[5]))
    rc.grabber_left_speed = np.uint8(l[6])
    rc.grabber_right_speed = np.uint8(l[7])

    return True
 
def main():
    initROS()
    initUDP()

    rate = rospy.Rate(50) 

    while not rospy.is_shutdown():

        m = ReceiveUDP()

        msg = RobotCommand()
        if not processUDPMessage(m, msg) == False:
            pub.publish(msg)
        
        ##rospy.loginfo(t)
        rate.sleep()

    s.close() #Close the udp socket

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
