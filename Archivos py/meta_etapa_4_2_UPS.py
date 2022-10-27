#!/usr/bin/env python3

#--------------------------------- Librerias ------------------------------------------

import rospy
import numpy as np
import time
import tf2_ros
import tf
import math
import smach
import ros_numpy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2
from geometry_msgs.msg import PoseStamped,Pose
from utils_evasion import *
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist, Point

#------------------------------ Inicializando Nodo ------------------------------------

rospy.init_node("nodo1")
EQUIPO="UPS Cuenca"
r=rospy.Rate(4)
rate = rospy.Rate(1)
twist=Twist()
global detect, ini, xp, yp, tol_1, tol_2, xr, yr, radio_xp, radio_yp, angle_to_goal, angle_to_goal1, inc_x, inc_y

tol_1, tol_2, xr, yr, radio_xp, radio_yp, angle_to_goal, angle_to_goal1, inc_x, inc_y=0, 0, 0, 0, 0, 0, 0, 0, 0, 0
def init(node_name):
    global laser, base_vel_pub
    base_vel_pub=rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10) 
    laser=Laser()
    rospy.Subscriber("/hsrb/odom",Odometry,queue_size=10)
    rate = rospy.Rate(1) # 1hz
    

#------------------------- Funcion Acondicionamiento Laser ---------------------------
        
def get_lectura_cuant():
    try:
        global right_scan, left_scan, front_scan
        lectura=np.asarray(laser.get_data().ranges)
        lectura=np.where(lectura>20,20,lectura) #remove infinito
        right_scan=lectura[:220] #180
        left_scan=lectura[540:] #540
        front_scan=lectura[220:540] #180:540
        #print('lf: '+ str(np.mean(front_scan)))
        #print('ll: '+ str(np.mean(left_scan)))
        sd,si,sf=0,0,0
        if np.mean(left_scan)< 1.15: si=1
        if np.mean(right_scan)< 2.0: sd=1
        if np.mean(front_scan)< 1.65: sf=1
    except:
        sd,si,sf=0,0,0    
    return si,sd,sf

#------------------------- Funciones Movimiento Robot --------------------------------

def move_base_vel(vx, vy, vw):
    twist.linear.x=vx
    twist.linear.y=vy
    twist.angular.z=vw 
    base_vel_pub.publish(twist)
def move_base(x,y,yaw,timeout):
    start_time=rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec()-start_time<timeout:  
        move_base_vel(x, y, yaw) 
        
#---------------------------------- Estados --------------------------------------    

class S0(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3','outcome4','outcome5','outcome6','outcome7','outcome8','outcome9'])
        self.counter=0
    def execute(self,userdata):
        si,sd,sf=get_lectura_cuant()
        msg_cmd_vel=Twist()
        tole_i, tole_f = 3.5, 1.5
        global xp, yp, tol_1, tol_2, xr, yr, left_scan, rigth_scan, theta_R, radio_xp, radio_yp, angle_to_goal, angle_to_goal1, inc_x, inc_y
        data = rospy.wait_for_message('/hsrb/odom', Odometry, timeout=10)
        xr=data.pose.pose.position.x
        yr=data.pose.pose.position.y
        zr=data.pose.pose.position.z
        xo=data.pose.pose.orientation.x
        yo=data.pose.pose.orientation.y
        zo=data.pose.pose.orientation.z
        wo=data.pose.pose.orientation.w  
        (roll, pitch, theta) = euler_from_quaternion ([xo, yo, zo, wo])   
        #rate.sleep()
        if (ini == 1) and (tol_1==0):
            radio_xp = abs(xr-xp)
            radio_yp = abs(yr-yp)
            print('radio_xp: ' + str(radio_xp))
            print('radio_xp: ' + str(radio_yp))
            if ( (radio_xp > tole_i) and (radio_yp > tole_i) ):
                tol_1=1
        
        if (tol_1 == 1): 
            radio_xp = abs(xr-xp)
            radio_yp = abs(yr-yp)
            print('radio_xp: ' + str(radio_xp))
            print('radio_xp: ' + str(radio_yp))
            if ((radio_xp < tole_f) and (radio_yp < tole_f)):
                tol_2=1
                
        
        print('xr: ' + str(xr))
        print('yr: ' + str(yr))
        print('tol_1: ' + str(tol_1))
        print('tol_2: ' + str(tol_2))
        
        if (np.mean(left_scan)<0.8 and tol_2 == 0):
            print('base apegada')
            move_base(0.0,0.0,-0.12*np.pi,2)
            
        if (np.mean(right_scan)<0.8 and tol_2 == 0):
            print('base apegada')
            move_base(0.0,0.0,0.12*np.pi,0.5)
        
        if (tol_2 == 1):
            inc_x = 0 - xr
            inc_y = 0 - yr
            angle_to_goal = atan2(inc_y, inc_x)
            angle_to_goal1 = angle_to_goal-theta
            
            print('inc_x: ' + str(inc_x))
            print('inc_y: ' + str(inc_y))
            print('angle_to_goal: ' + str(angle_to_goal))
            print('angle_to_goal1: ' + str(angle_to_goal1))
            
        if (si==0 and sf==0 and sd==1): return 'outcome2'
        if (si==0 and sf==1 and sd==0): return 'outcome3'
        if (si==0 and sf==1 and sd==1): return 'outcome4'
        if (si==1 and sf==0 and sd==0): return 'outcome5'
        if (si==1 and sf==0 and sd==1): return 'outcome6'
        if (si==1 and sf==1 and sd==0): return 'outcome7'
        if (si==1 and sf==1 and sd==1): return 'outcome8'
        if (si==0 and sd==0 and sf==0): return 'outcome9' 
        return 'outcome1' 
        pub_cmd_vel.publish(msg_cmd_vel)
        r.sleep() 

class S1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter=0
    def execute(self,userdata):
        rospy.loginfo('Estado Derecha')
        global detect,ini, xp, yp, tol_2, angle_to_goal, angle_to_goal1, inc_x, inc_y
        detect=1
        if (ini==0):
            data = rospy.wait_for_message('/hsrb/odom', Odometry, timeout=10)
            xp=data.pose.pose.position.x
            yp=data.pose.pose.position.y
            ini = 1
        if (tol_2 == 0):
            move_base(0.1,0.0,0.12*np.pi,0.1)
        else:
            if abs(inc_x)>0.05 and abs(inc_y)>0.05:
                move_base(0.3,0.0,0.12*np.pi,0.08)
        return 'outcome1'

class S2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter=0
    def execute(self,userdata):
        rospy.loginfo('Estado Frente')
        global detect,ini, xp, yp, angle_to_goal, angle_to_goal1, inc_x, inc_y
        detect=1
        if (ini==0):
            data = rospy.wait_for_message('/hsrb/odom', Odometry, timeout=10)
            xp=data.pose.pose.position.x
            yp=data.pose.pose.position.y
            ini = 1
        if (tol_2 == 0):
            move_base(0.0,0.0,-0.12*np.pi,2)
        else:
            if abs(inc_x)>0.05 and abs(inc_y)>0.05:
                if ((angle_to_goal1>=0 and angle_to_goal1<np.pi) or (angle_to_goal1<-np.pi)):
                    if abs(angle_to_goal1)>0.1: move_base(0.0,0.0,0.12*np.pi,0.1)                 
                elif ((angle_to_goal1>=np.pi and angle_to_goal1<2*np.pi) or (angle_to_goal1<0 and angle_to_goal1>-(np.pi))):
                    if abs(angle_to_goal1)>0.1: move_base(0.0,0.0,-0.12*np.pi,0.1)
        return 'outcome1'

class S3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter=0
    def execute(self,userdata):
        rospy.loginfo('Estado Frente - Derecha')
        global detect,ini, xp, yp, angle_to_goal, angle_to_goal1, inc_x, inc_y
        detect=1
        if (ini==0):
            data = rospy.wait_for_message('/hsrb/odom', Odometry, timeout=10)
            xp=data.pose.pose.position.x
            yp=data.pose.pose.position.y
            ini = 1
        if (tol_2 == 0):
            move_base(0.0,0.0,0.12*np.pi,0.1)
        else:
            if abs(inc_x)>0.05 and abs(inc_y)>0.05:
                move_base(0.0,0.0,0.12*np.pi,0.1)
        return 'outcome1'

class S4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter=0
    def execute(self,userdata):
        rospy.loginfo('Estado Izquierda')
        global detect,ini, xp, yp, angle_to_goal, angle_to_goal1, inc_x, inc_y
        detect=1
        if (ini==0):
            data = rospy.wait_for_message('/hsrb/odom', Odometry, timeout=10)
            xp=data.pose.pose.position.x
            yp=data.pose.pose.position.y
            ini = 1
        if (tol_2 == 0):
            move_base(0.3,0.0,-0.07*np.pi,0.1)
        else:
            if abs(inc_x)>0.05 and abs(inc_y)>0.05:
                move_base(0.3,0.0,-0.12*np.pi,0.08)
        return 'outcome1' 

class S5(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter=0
    def execute(self,userdata):
        rospy.loginfo('Estado Izquierda - Derecha')
        global detect,ini, xp, yp, angle_to_goal, angle_to_goal1, inc_x, inc_y
        detect=1
        if (ini==0):
            data = rospy.wait_for_message('/hsrb/odom', Odometry, timeout=10)
            xp=data.pose.pose.position.x
            yp=data.pose.pose.position.y
            ini = 1
        if (tol_2 == 0):
            move_base(0.3,0,-0.003*np.pi,0.08)
        else:
            move_base(0.3,0,0,0.1)
        return 'outcome1' 

class S6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter=0
    def execute(self,userdata):
        rospy.loginfo('Estado Frente - Izquierda')
        global detect,ini, xp, yp, angle_to_goal, angle_to_goal1, inc_x, inc_y
        detect=1
        if (ini==0):
            data = rospy.wait_for_message('/hsrb/odom', Odometry, timeout=10)
            xp=data.pose.pose.position.x
            yp=data.pose.pose.position.y
            ini = 1
        if (tol_2 == 0):
            move_base(0.0,0.0,-0.12*np.pi,3.3)
        else:
            if abs(inc_x)>0.05 and abs(inc_y)>0.05:
                move_base(0.0,0.0,-0.12*np.pi,0.1)
        return 'outcome1' 

class S7(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter=0
    def execute(self,userdata):
        rospy.loginfo('Estado Reversa')
        global detect, ini, xp, yp, angle_to_goal, angle_to_goal1, inc_x, inc_y
        detect=1
        if (ini==0):
            data = rospy.wait_for_message('/hsrb/odom', Odometry, timeout=10)
            xp=data.pose.pose.position.x
            yp=data.pose.pose.position.y
            ini = 1
        if (tol_2 == 0):
            move_base(0.1,0,-0.12*np.pi,0.08)
        else:
            move_base(-0.3,0,0,0.1)
        return 'outcome1' 

class S8(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter=0
    def execute(self,userdata):
        rospy.loginfo('Estado Comparacion')
        global detect,ini, angle_to_goal, angle_to_goal1, inc_x, inc_y
        if (tol_2 == 0):
            if (detect == 0):
                move_base(0.3,0.0,-0.01*np.pi,0.1)
            if (detect == 1):
                move_base(0.1,0.0,0.07*np.pi,0.08)
                move_base(0.07,0.0,0.0,0.1)
        else:
            if abs(inc_x)<0.05 and abs(inc_y)<0.05:
            	#rosrun map_server map_saver en python
                return 'outcome2'
            else:
                if ((angle_to_goal1>=0 and angle_to_goal1<np.pi) or (angle_to_goal1<-np.pi)):
                    if abs(angle_to_goal1)>0.1: move_base(0.0,0.0,0.12*np.pi,0.1)                 
                    if abs(angle_to_goal1)<0.1: move_base(0.3,0,0,0.1)
                elif ((angle_to_goal1>=np.pi and angle_to_goal1<2*np.pi) or (angle_to_goal1<0 and angle_to_goal1>-(np.pi))):
                    if abs(angle_to_goal1)>0.1: move_base(0.0,0.0,-0.12*np.pi,0.1)  
                    if abs(angle_to_goal1)<0.1: move_base(0.3,0,0,0.1)
        return 'outcome1'

def main():
    global pub_cmd_vel, detect,ini
    print("Meta Competencia - " + EQUIPO)
    pub_cmd_vel=rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
    loop = rospy.Rate(10)
    print('Inicializando')
    detect=0
    ini=0
    rospy.sleep(1)
    
    
if __name__ == '__main__':
    init("takeshi_smach")
    sm=smach.StateMachine(outcomes=['END'])     #State machine, final state "END"
    sm.userdata.sm_counter=0
    sm.userdata.clear=False   
    with sm:
        smach.StateMachine.add("s_0",   S0(),  transitions = {'outcome1':'s_0', 'outcome2':'s_1','outcome3':'s_2','outcome4':'s_3','outcome5':'s_4', 'outcome6':'s_5','outcome7':'s_6','outcome8':'s_7','outcome9':'s_8',})
        smach.StateMachine.add("s_1",   S1(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_2",   S2(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_3",   S3(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_4",   S4(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_5",   S5(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_6",   S6(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_7",   S7(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_8",   S8(),  transitions = {'outcome1':'s_0','outcome2':'END'})
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        
outcome=sm.execute()
    
