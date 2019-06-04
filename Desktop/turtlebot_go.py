#!/usr/bin/env python

#  Turtlebot Project1:		1. Move the turtlebot to the specific goal pose 
#                             	2. Waypoint Following: Move along some basic closed shape
#                                  (so that it theoretically will end where it started)

# A basic script to make the turtlebot move forwards for 4 seconds. Press CTRL + C to stop.  
# To run:
# 	On TurtleBot:
# 		$roslaunch turtlebot3_bringup turtlebot3_robot.launch --screen
# 	On Remote PC:
# 		$python turtlebot_go.py

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
	wp = [[0,0,0],[3,0,0],[3,3,math.pi/2],[0,0,math.pi]]
	next_pos =[3, 3, 45*math.pi/180]
	prev_pos =[-0.01, 0, 0]
	wp_idx = 1
	vel = 0.5
	#psi = 0.0
	while(wp_idx<len(wp)):
		next_pos =wp[wp_idx]
		prev_pos =wp[wp_idx-1]
		while (abs(self.x - next_pos[0]) > 0.2 or abs(self.y - next_pos[1]) > 0.2 or abs(self.theta - next_pos[2]) > 1.0):
			if (time.time() - t0 > 0.2):
				move_cmd = Twist()

				b = [next_pos[0]-prev_pos[0], next_pos[1]-prev_pos[1]]
				a = [self.x-prev_pos[0], self.y-prev_pos[1]]
				det = a[0]*b[1] - a[1]*b[0]
				abcos = (a[0]*b[0]+a[1]*b[1])/(math.sqrt(a[0]**2+a[1]**2)*math.sqrt(b[0]**2+b[1]**2))

				if(abcos>=1):
					psi = 0
				elif(abcos<=-1):
					psi = math.pi
				else:
					psi = det/abs(det)*math.acos(abcos)

				theta_p = math.atan(b[1]/b[0])
				theta_e = self.theta-theta_p
				theta_e = theta_e%(2*math.pi)

				if theta_e > math.pi:
					theta_e = theta_e - 2*math.pi
				elif theta_e < -math.pi:
					theta_e = theta_e + 2*math.pi
	
				e_ld = math.sqrt(a[0]**2+a[1]**2)*math.sin(psi)
				k_ld = 0.1
			
				ld = k_ld*vel
				ld_min = 0.2
				ld_max = 2
				if ld >= ld_max:
					ld = ld_max
				elif ld<=ld_min:
					ld = ld_min

				across=e_ld/ld
				print("across = [%.2f]"%(across))
				if across>=1:
					across=1.0
				elif across<=-1:
					across=-1.0
				k_steer = 0.8
				theta_g = math.asin(across)
				alpha = theta_g-theta_e

				c = b-a
				if(math.sqrt(c[0]**2+c[1]**2>1):
					vel = 2
				elif(math.sqrt(c[0]**2+c[1]**2>0):
					vel = 0.5
				delta = k_steer*alpha
				move_cmd.angular.z = delta
				move_cmd.linear.x = vel

				#update turtlebot's pos by odometry
			

			
				print("[%.2f]x y theta= %.2f %.2f %.2f"%(vel,self.x,self.y,self.theta/math.pi*180))
			#turn left when positive value
			    # Publish the commands to the turtlebot node then wait for the next cycle
			    	#move_cmd.angular.z = move_cmd.angular.z * 1.33 # Offset to correct the rotational speed

				self.cmd_vel.publish(move_cmd)
				t0=time.time()
		wp_idx +=1

#	r.sleep()
        
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
	self.theta = math.atan2(siny,cosy)
		
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

