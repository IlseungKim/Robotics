#!/usr/bin/env python

#  Turtlebot Project1:      1. Move the turtlebot to the specific goal pose
#                                2. Waypoint Following: Move along some basic closed shape
#                                  (so that it theoretically will end where it started)

# A basic script to make the turtlebot move forwards for 4 seconds. Press CTRL + C to stop.  
# To run:
#    On TurtleBot:
#       $roslaunch turtlebot3_bringup turtlebot3_robot.launch --screen
#    On Remote PC:
#       $python turtlebot_go.py

import rospy
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState
from nav_msgs.msg import Odometry
import time
import math
import csv

class DrawShape():
    x = 0
    y = 0
    theta = 0
    def __init__(self):
        # initiliaze
        rospy.init_node('Turtlebot_Project1', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Open a file to write all the sensor values to
        self.file = open('encoderValues.csv','w')
        self.file.write('Left Encoder,Right Encoder\n')

        # Subscribe to the sensor core topic to read the encoder values
        self.subscriber = rospy.Subscriber("/mobile_base/sensors/core",SensorState,self.writeData)
        self.subscriber_odom = rospy.Subscriber('odom',Odometry,self.Position)

        #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10)
        t0 = time.time()

        ######################################################
        #----------------Modify this code---------------------
        # Draw out some basic shape
        # NOTE: linear is in m/s and angular is meant to be in
        # rad/s but moves at only around 75% intended speed,
        # which is compensated for by the factor of 1.33 applied
        ######################################################

        # Here we'll loop through a series of commands
        odom = Odometry()  
        #waypoint를 설정한다. 주어진 waypoint는 한 변이 1미터인 사각형그리고 제자리로 돌아오게 하였다.
        wp = [[-0.1,0,0],[1,1,math.pi],[0,1,math.pi],[0,0,-math.pi/2]]
        #next_pos와 prev_pos를 설정하기 위하여 index를 1로 설정한다.
        wp_idx = 1

        #waypoint와 충분히 가까워지면 속도에 0.5, 빠르게 주행할 상황에는 2를 사용한다.
        vel = 0.5
        fast_vel = 2
        #모든 waypoint를 통과할때까지 while문 실행한다.
        while(wp_idx < len(wp)):
            #next_pos는 도달해야할 waypoint, prev_pos는 이미 도달한 바로 직전 waypoint의 좌표값이다.
            next_pos = wp[wp_idx]
            prev_pos = wp[wp_idx-1]
            #각 waypoint에 도달할 때까지 while문을 실행한다.
            #waypoint에 도달할 조건은 waypoint와 현재 로봇의 [x,y]좌표의 차이가 각각 0.2, 로봇의 orientation의 차이가 1 radian 이내 일 때 이다.
            while (abs(self.x - next_pos[0]) > 0.2 or abs(self.y - next_pos[1]) > 0.2 or (abs(self.theta) - abs(next_pos[2])) > 1.0):
                #로봇이 명령을 받고 작동을 할 시간을 0.1초로 설정
                if (time.time() - t0 > 0.1):
                    move_cmd = Twist()

                    #b vector는 이전 waypoint에서 다음 waypoint로의 벡터이다.
                    b = [next_pos[0]-prev_pos[0], next_pos[1]-prev_pos[1]]
                    #a vector는 이전 waypoint에서 로봇의 좌표로의 벡터이다.
                    a = [self.x-prev_pos[0], self.y-prev_pos[1]]
                    
                    #abcos은 벡터 a,b의 사잇각이다.
                    det = a[0]*b[1] - a[1]*b[0]
                    abcos = (a[0]*b[0]+a[1]*b[1])/(math.sqrt(a[0]**2+a[1]**2)*math.sqrt(b[0]**2+b[1]**2))

                    #abcos을 -pi~pi범위로 한정하고 벡터 b를 기준으로 벡터 a의 각도를 구하기위해 부호를 계산한다.
                    if(abcos>=1):
                        psi = 0
                    elif(abcos<=-1):
                        psi = math.pi
                    else:
                        psi = det/abs(det)*math.acos(abcos)

                    #theta_p는 벡터 b와 x축 사이의 각도이다.
                    theta_p = math.atan2(b[1],b[0])
                    #theta_e는 벡터 b와 로봇의 방향 사이의 각도이다.
                    theta_e = self.theta-theta_p
                    
                    #theta_e의 값을 -pi~pi 범위로 한정한다.
                    theta_e = theta_e%(2*math.pi)
                    if theta_e > math.pi:
                        theta_e = theta_e - 2*math.pi
                    elif theta_e < -math.pi:
                        theta_e = theta_e + 2*math.pi

                    #e_ld는 로봇에서 b벡터에 내린 수선의 벡터이다.
                    e_ld = math.sqrt(a[0]**2+a[1]**2)*math.sin(psi)
                    #k_ld는 로봇의 속도에 곱해는 gain값이다.
                    k_ld = 1
                    #ld는 look ahead distance이다.
                    ld = k_ld*vel
                    ld_min = 0.5
                    ld_max = 2
                    
                    if ld >= ld_max:
                        ld = ld_max
                    elif ld<=ld_min:
                        ld = ld_min

                    #벡터 b와 ld가 이루는 각도인 theta_g를 계산한다. across는 asin의 정의역의 범위에 맞춰
                    #-1~1 범위로 한정한다.
                    across=e_ld/ld
                    print("across = [%.2f]"%(across))
                    if across>=1:
                        across=1.0
                    elif across<=-1:
                        across=-1.0
                    k_steer = 1
                    theta_g = math.asin(across)
                    
                    #alpha는 벡터 b와 로봇의 진행방향이 이루는 각이다. 이 각도에 비례하여 steer을 결정한다.
                    alpha = theta_g-theta_e

                    c = [b[0]-a[0], b[1]-a[1]]
                    if(math.sqrt(c[0]**2+c[1]**2)>2):
                        vel = 1.0
                    elif(math.sqrt(c[0]**2+c[1]**2)>1):
                        vel = 0.8
                    elif(math.sqrt(c[0]**2+c[1]**2)>0):
                        vel = 0.3
                    elif(math.sqrt(c[0]**2+c[1]**2)<0.2 and abs(across)<0.3):
                        vel = 0.0
                    delta = k_steer*alpha
                    move_cmd.angular.z = delta
                    move_cmd.linear.x = vel

                    #update turtlebot's pos by odometry
                   
                
                    print("[%.2f]x y theta= %.2f %.2f %.2f"%(vel,self.x,self.y,self.theta/math.pi*180),(next_pos[2]/math.pi*180))
        #turn left when positive value
            # Publish the commands to the turtlebot node then wait for the next cycle
                #move_cmd.angular.z = move_cmd.angular.z * 1.33 # Offset to correct the rotational speed

                    self.cmd_vel.publish(move_cmd)
                    t0=time.time()
            wp_idx += 1
            print("ARRIVED")

#   r.sleep()
       
       ######################################################
   
   # Write data from the encoders to the csv we opened
    def writeData(self,data):
        self.file.write('%(left)d,%(right)d\n' % {'left': data.left_encoder,'right': data.right_encoder})
    def Position(self,data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        w = data.pose.pose.orientation.w
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        siny = 2.0*(w*z+y*x)
        cosy = 1.0 - 2.0*(y**2+z**2)
        tmpp = math.atan2(siny,cosy)
        if(tmpp <= -177.0):
            self.theta = -tmpp
        else:
            self.theta = tmpp
     
   # Unsubscribe from the sensor topic and tell the turtlebot to stop moving
    def shutdown(self):
        self.subscriber.unregister()
        self.file.close()
        rospy.loginfo("Stop TurtleBot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
 #  try:
    DrawShape()
#    except:
#       rospy.loginfo("DrawShape node terminated.")