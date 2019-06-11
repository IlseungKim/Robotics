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
    y = 1
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
	
	wp =[[3,0,0],[3,3,90],[0,3,180],[0,0,-90]]	
	
	wp_idx =1
	while(wp_idx<len(wp)):
		next_pos =wp[wp_idx]
		prev_pos =wp[wp_idx-1]
		while (abs(self.x - next_pos[0]) > 0.2 or abs(self.y - next_pos[1]) > 0.2 or abs(self.theta - next_pos[2]) > 5.0):
#			if (time.time() - t0 > 0.2):
			move_cmd = Twist()

			k_rho = 0.3
			k_alpha = 0.15
			k_beta =-0.0

			rho = math.sqrt((next_pos[1]-self.y)**2+(next_pos[0]-self.x)**2)
			lamb = math.atan2(next_pos[1]-self.y,next_pos[0]-self.x)
			alpha = lamb - self.theta
			alpha = alpha % (2*math.pi)
			if alpha > math.pi:
				alpha = alpha - 2*math.pi
			elif alpha < -math.pi:
				alpha = alpha + 2*math.pi
		
			beta = -self.theta - alpha
			beta = beta % (2*math.pi)
			if beta > math.pi:
				beta = beta - 2*math.pi
			elif beta < -math.pi:
				beta = beta + 2*math.pi

			if rho < 0.2:
				rho = 0.2
			move_cmd.angular.z = alpha*k_alpha + beta*k_beta
			move_cmd.linear.x = rho*k_rho
					
			print("[%.2f]x y theta= %.2f %.2f %.2f %.2f"%(rho*k_rho,self.x,self.y,alpha,beta))
		#turn left when positive value
		    # Publish the commands to the turtlebot node then wait for the next cycle
		    	#move_cmd.angular.z = move_cmd.angular.z * 1.33 # Offset to correct the rotational speed

			self.cmd_vel.publish(move_cmd)
#			t0=time.time()
        
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
	self.theta = math.atan2(siny,cosy)/math.pi*180
		
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

