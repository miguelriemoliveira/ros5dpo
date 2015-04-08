#!/usr/bin/env python2
# license removed for brevity
import rospy
from math import sqrt
from math import pow
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from rtdb2ros.msg import RobotState
from nav_msgs.msg import Odometry
import socket
import sys
import numpy as np
import math
from tf.transformations import euler_from_quaternion
#from tf.transformations import quaternion
 
def ReceiveUDP():
    bdata = bytearray(4096)       
    nbytes, sender = s_receive.recvfrom_into(bdata, 100)
    #print "Received " + str(nbytes) + " bytes"

    l = list()
    l.append(str(bdata[0]))
    l.append(chr(bdata[1]))
    for i in range(2,nbytes):
        l.append(str(bdata[i]))

    print l
    return l      

class CommandMotors:
    encoded_w = [0,0,0] #motor 1 velocity in ticks'
    w = [0,0,0]  #motor 1 velocity in rad/sec
    v = [0,0,0] #high level vx,vy,vw
    
    previous_v = [0,0,0]
    delta_v = [0,0,0]

    KSamplePeriodRaw2Processed=10; #Copied from Decision code
    KTicks2Rad = 0.0005; #Copied from Decision code
    WheelsRadius = 0.0513 #Copied from Decision code
    WheelToCenterDist = 0.1885 #Copied from Decision code
    EncAbs = [0,0,0]
    EncReceivedFirst = 0

    
    def __str__(self):
		l = [self.encoded_w, self.w, self.v]
		return str(l)

    def EncodedToW(self):
        for i in range(0,3):
            self.w[i] = self.encoded_w[i] * self.KTicks2Rad /10 * 1000 * self.WheelsRadius;  

    def WToEncoded(self):
        for i in range(0,3):
            #self.w[i] = self.encoded_w[i] * self.KTicks2Rad /10 * 1000 * self.WheelsRadius;  
            #self.encoded_w[i] = self.w[i] / self.KTicks2Rad *10 / 1000 / self.WheelsRadius;  
            self.encoded_w[i] = self.w[i] / self.KTicks2Rad / self.WheelsRadius;  
            #self.encoded_w[i] = np.int16(round(self.w[i] / self.KTicks2Rad*(self.KSamplePeriodRaw2Processed/1000), 0));
            #self.encoded_w[i] = np.int16(self.w[i] / self.KTicks2Rad);
            #FMotorRaw[MotorNum].OutputRef:=Smallint(round(FMotorProcessed[MotorNum].AngVelRef / KTicks2Rad*(KSamplePeriodRaw2Processed/1000)));

    def WToVxVyVW(self):
    #Following paper http://cdn.intechopen.com/pdfs-wm/8875.pdf
        self.v[0] = 0.5774 * (self.w[0] - self.w[1])
        self.v[1] = 0.3333 * (self.w[0] + self.w[1]) - 0.6666 * self.w[2]
        self.v[2] = (0.3333 / self.WheelToCenterDist) * (self.w[0] + self.w[1] + self.w[2])

    def VxVyVWToW(self):
        self.w[0] = 0.886*self.delta_v[0] + 0.5*self.delta_v[1] + self.WheelToCenterDist*self.delta_v[2];
        self.w[1] = -0.886*self.delta_v[0] + 0.5*self.delta_v[1] + self.WheelToCenterDist*self.delta_v[2];
        self.w[2] = -1*self.delta_v[1] + self.WheelToCenterDist * self.delta_v[2];

        #speed2w:=-0.886*NewMotioncommand.V+0.5*NewMotioncommand.Vn+FGeometryParameters.Wheels2Centerdist*NewMotioncommand.W;
        #speed3w:=-1*NewMotioncommand.Vn+FGeometryParameters.Wheels2Centerdist*NewMotioncommand.w;

    def EncodedToUDPMsg(self):
        print self.encoded_w
        #msg_out = []
        #msg_out.append(0)
        #msg_out.append('R')

        for i in range(0,3):
            #msg_out.append(i+1)
            #msg_out.append(6) #memory position
            #msg_out.append(2) #bytes to write

            #self.encoded_w[i]

            #value = np.uint16(((lsb) << 8)  | msb)

            value = self.encoded_w[i]
            
            #value = 0
            self.EncAbs[i] += value

            #self.EncAbs[1] =  self.EncAbs[0]
            #self.EncAbs[2] =  self.EncAbs[0]
            #lsb = bytearray(1)
            #msb = bytearray(1)
            msb = np.uint8(np.uint16(self.EncAbs[i]) & 0b0000000011111111)
            #msb=0
            lsb = np.uint8((np.uint16(self.EncAbs[i]) >> 8))
            #lsb = 0
            #msg_out.append(lsb)  
            #msg_out.append(msb)  

            bdata = bytearray(11)       
            bdata[0] = i+1
            bdata[1] = 82
            #bdata[1] = str("R")
            bdata[2] = 5
            bdata[3] = 0
            #bdata[4] = 1 #msb
            #bdata[5] = 1 #lsb
            bdata[4] = lsb #msb
            bdata[5] = msb #lsb
            bdata[6] = 0
            bdata[7] = 0
            bdata[8] = 0
            bdata[9] = 0
            bdata[10] = 0

            s.sendto(bdata , (SENT_HOST,SENDPORT))

        print "EncAbs " + str(self.EncAbs)
        print "uint16 EncAbs " + str(np.uint16(self.EncAbs))

        #msg_out = str("1R607")
        #msg_out = str("1R607")


        #for i in range(1,4):


        #print msg_out

        #s_send.sendto(str(msg_out) , (HOST,SENDPORT))
        #s.sendto(str(msg_out) , (HOST,SENDPORT))
        #s.sendto(msg_out , (HOST,SENDPORT))

    def UDPMsgToEncoded(self, msg):
        #print str(msg)
        if not msg[1] == 'W' and not msg[0] == 0: #only process write msgs
            return

        #print "Processing " + str(msg) + " size " + str(len(msg))
		
        i = 2
        #for i in range(2,len(msg)):
        while i<len(msg):

            #print " at i=" + str(i)
            slave = -1
            if msg[i] == '1':
                slave = 1
            elif msg[i] == '2':
                slave = 2
            elif msg[i] == '3':
                slave = 3

            i += 1
            address = msg[i]
            i += 1
            nbytes = msg[i]
            i += 1
            #print "slave = " + str(slave) + " address " + str(address) + " nbytes " + str(nbytes)


            if address == '0' and nbytes == '3':
                value = int()
                lsb = int(msg[i+1])
                msb = int(msg[i+2])

                #msb = 0b01000000
                #lsb = 0b00001001
                #print str(lsb) + " " + str(msb)
                value = np.int16(((lsb) << 8)  | msb)
                self.encoded_w[slave-1] = value
                #print " value " + str(value)

			#jump the bytes to read
            #i += int(nbytes)
            #print " jumped to i=" + str(i)

            
#Global Variables       
LOCAL_HOST = '127.0.0.1'   # Symbolic name meaning all available interfaces
#SENT_HOST = '172.16.33.100'   # Symbolic name meaning all available interfaces
SENT_HOST = '127.0.0.1'   # Symbolic name meaning all available interfaces
PORT = 5000 # Arbitrary non-privileged port
SENDPORT = 5001 # Arbitrary non-privileged port
SENDPORT_POSE = 5002 # Arbitrary non-privileged port
SENDPORT_BALL = 5003 # Arbitrary non-privileged port
cmd_motors = CommandMotors()
odometry = CommandMotors()
odometry.EncAbs[0] = np.uint16(0);
odometry.EncAbs[1] = np.uint16(0);
odometry.EncAbs[2] = np.uint16(0);

s = socket.socket()
s_receive = socket.socket()
#s_send = socket.socket()
pub = []
robot_pose_x = 0
robot_pose_y = 0
max_ball_dist = 2
 
def robotStateCallback(data):
    print "I am in a robotStateCallback"
    global robot_pose_x 
    global robot_pose_y 

    robot_pose_x = data.agent_pose.pose.position.x;
    robot_pose_y = data.agent_pose.pose.position.y;

    msg = []
    msg.append(round(-data.agent_pose.pose.position.y,3))
    msg.append(round(data.agent_pose.pose.position.x,3))
    #quaternion q
    q =  np.array([data.agent_pose.pose.orientation.x, data.agent_pose.pose.orientation.y, data.agent_pose.pose.orientation.z, data.agent_pose.pose.orientation.w])
    (roll,pitch,yaw) = euler_from_quaternion(q);
    msg.append(round(yaw+3.14/2,3))

    #msg_str = str(msg)
    #if len(msg_str)>3:
        #msg_str = msg_str[1:len(msg_str)-1]

    #print msg_str
    #print SENT_HOST
    #print SENDPORT_POSE
    #if not len(msg_str)==0:
    #s.sendto(msg_str, (SENT_HOST,SENDPORT_POSE))

    ## FOR THE BALL
    #msg = []
    msg.append(round(-data.ball_pose.pose.position.y,3))
    msg.append(round(data.ball_pose.pose.position.x,3))

    dist = math.sqrt(math.pow(robot_pose_x - round(data.ball_pose.pose.position.x,3),2) +
            math.pow(robot_pose_y - round(data.ball_pose.pose.position.y,3),2))

    if dist < max_ball_dist:
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
    s.sendto(msg_str , (SENT_HOST,SENDPORT_BALL))


def poseCallback(data):
    global robot_pose_x 
    global robot_pose_y 

    robot_pose_x = data.pose.position.x;
    robot_pose_y = data.pose.position.y;

    msg = []
    msg.append(round(-data.pose.position.y,3))
    msg.append(round(data.pose.position.x,3))
    #quaternion q
    q =  np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    (roll,pitch,yaw) = euler_from_quaternion(q);
    msg.append(round(yaw+3.14/2,3))

    msg_str = str(msg)
    if len(msg_str)>3:
        msg_str = msg_str[1:len(msg_str)-1]

    #print msg_str
    #print SENT_HOST
    #print SENDPORT_POSE
    #if not len(msg_str)==0:
    s.sendto(msg_str, (SENT_HOST,SENDPORT_POSE))

def ballPoseCallback(data):
    msg = []
    msg.append(round(-data.pose.position.y,3))
    msg.append(round(data.pose.position.x,3))
    #quaternion q
    #q =  np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    #(roll,pitch,yaw) = euler_from_quaternion(q);

    dist = math.sqrt(math.pow(robot_pose_x - round(data.pose.position.x,3),2) +
            math.pow(robot_pose_y - round(data.pose.position.y,3),2))

    if dist < max_ball_dist:
        msg.append(1)
    else:
        msg.append(0)

    #Remove the first and last elements []
    msg_str = str(msg)
    if len(msg_str)>3:
        msg_str = msg_str[1:len(msg_str)-1]
    s.sendto(msg_str , (SENT_HOST,SENDPORT_BALL))



def odomCallback(data):
    return #for now do nothing with the odom callback
    #rospy.loginfo(data)
    print "EncAbs " + str(odometry.EncAbs)
    #wait = input("PRESS ENTER TO CONTINUE.")
    #data.pose.pose.orientation.z -= math.pi

    print "new callback"
    if odometry.EncReceivedFirst == 0:
        print "First encoder msg received"
        odometry.EncReceivedFirst = 1
        odometry.previous_v[0] = data.pose.pose.position.x
        odometry.previous_v[1] = data.pose.pose.position.y
        odometry.previous_v[2] = data.pose.pose.orientation.z
        return


    #set the delta_v and previous_v
    odometry.delta_v[0] = data.pose.pose.position.x - odometry.previous_v[0];
    odometry.delta_v[1] = data.pose.pose.position.y - odometry.previous_v[1];

    #odometry.delta_v[2] = odometry.delta_v[2] = (data.pose.pose.orientation.z - math.pi) - odometry.previous_v[2];


    diff = abs(data.pose.pose.orientation.z  - odometry.previous_v[2]);
    print "previous v = " + str(odometry.previous_v)
    print "diff = " + str(diff)
    if diff >= math.pi:
        print "detected angle cycle"
        if data.pose.pose.orientation.z > odometry.previous_v[2]:
            d1 = 2*math.pi - data.pose.pose.orientation.z
            d2 = odometry.previous_v[2]
            odometry.delta_v[2] = d1 + d2;
        else:
            d1 = data.pose.pose.orientation.z
            d2 = 2*math.pi - odometry.previous_v[2]
            odometry.delta_v[2] = d1 + d2;
    else:
        odometry.delta_v[2] = data.pose.pose.orientation.z - odometry.previous_v[2];

    #odometry.delta_v[2] = data.pose.pose.orientation.z - odometry.previous_v[2];

    odometry.previous_v[0] = data.pose.pose.position.x
    odometry.previous_v[1] = data.pose.pose.position.y
    odometry.previous_v[2] = data.pose.pose.orientation.z

    print "v " + str(odometry.previous_v)
    print "delta_v " + str(odometry.delta_v)
    odometry.VxVyVWToW()
    print "w " + str(odometry.w)
    odometry.WToEncoded()
    print "encoded " + str(odometry.encoded_w)
    odometry.EncodedToUDPMsg()


def initUDP():
    # Datagram (udp) socket
    try :
        global s
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print 'Socket created'
    except socket.error, msg :
        print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        sys.exit()
     
    # Bind socket to local host and port
    #try:
        #s.bind((LOCAL_HOST, PORT))
    #except socket.error , msg:
        #print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        sys.exit()
         
    print 'Socket bind complete. Listen port=' + str(PORT) + " send port=" + str(SENDPORT) + " send port_pose=" + str(SENDPORT_POSE) + " send port_ball=" + str(SENDPORT_BALL) 

    try :
        global s_receive
        s_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print 'Socket created'
    except socket.error, msg :
        print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        sys.exit()
     
    #Bind socket to local host and port
    try:
        #s_receive.bind((LOCAL_HOST, PORT))
        s_receive.bind(('', PORT))
    except socket.error , msg:
        print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]

    #try :
        #global s_send
        #s_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #print 'Socket created'
    #except socket.error, msg :
        #print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        #sys.exit()

    ## Bind socket to local host and port
    #try:
        #s_send.bind((HOST, SENDPORT))
    #except socket.error , msg:
        #print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        #sys.exit()
         
    print 'Socket bind complete'



def initROS():
    global pub

    rospy.init_node('msldecision2ros_node', anonymous=True)
    ns = rospy.get_namespace()
    print "namespace " + ns
    print "len " + str(len(ns))

    agent_number = 1

    if len(ns)>1:

        try:
            agent_number = int(ns[1])
        except ValueError:
            print 'Cannot define agent number from namespace. Using default 1'

    else:
        print 'Cannot define agent number from namespace. Using default 1'


    print "agent number " + str(agent_number)

    global PORT
    global SENDPORT 
    global SENDPORT_POSE
    global SENDPORT_BALL
    PORT = 5000 + (agent_number-1)*10
    SENDPORT = 5001  + (agent_number-1)*10
    SENDPORT_POSE = 5002  + (agent_number-1)*10
    SENDPORT_BALL = 5003  + (agent_number-1)*10


    pub = rospy.Publisher('velocity_cmd', Twist, queue_size=10)
    rospy.Subscriber("odom", Odometry, odomCallback)
    #rospy.Subscriber("pose_agent", PoseStamped, poseCallback)
    #rospy.Subscriber("pose_ball", PoseStamped, ballPoseCallback)
    rospy.Subscriber("robot_state", RobotState, robotStateCallback)
 
def main():
    initROS()
    initUDP()
    global max_ball_dist
    max_ball_dist = rospy.get_param('~max_ball_distance')   
    print "max_ball_distance=" + str(max_ball_dist)

    rate = rospy.Rate(50) 

    while not rospy.is_shutdown():

        l = ReceiveUDP()
            
        cmd_motors.UDPMsgToEncoded(l)
        cmd_motors.EncodedToW()
        cmd_motors.WToVxVyVW()
        #print "Command motors " + str(cmd_motors)

        #hello_str = "hello world %s" % rospy.get_time()
        t = Twist()
        t.linear.x = cmd_motors.v[0];
        t.linear.y = cmd_motors.v[1];
        t.linear.z = 0.0;

        t.angular.x = 0;
        t.angular.y = 0;
        t.angular.z = cmd_motors.v[2];

        #rospy.loginfo(t)
        pub.publish(t)
        rate.sleep()

    s.close() #Close the udp socket

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

                #def listener():

                        ## In ROS, nodes are uniquely named. If two nodes with
                    ## the same
                        ## node are launched, the previous one is kicked off. The
                        ## anonymous=True flag means that rospy will choose a
                    ## unique
                        ## name for our 'listener' node so that multiple
                    ## listeners can
                        ## run simultaneously.
                        #rospy.init_node('listener', anonymous=True)


                                ## spin() simply keeps python from exiting until
                            ## this node is stopped
                                #rospy.spin()

    #if __name__ == '__main__':
        #listener()
