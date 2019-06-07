#! /usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
import math
import pickle
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2


class pubVel():

	def __init__(self):

		rospy.init_node('pubVel', anonymous=False)

		rospy.loginfo("CTRL+C to stop the bot")

		rospy.on_shutdown(self.shutdown)

		self.odom_sub = rospy.Subscriber('/odom',Odometry,self.callback_odometry)

		self.pub_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

		self.vel = Twist()

		self.pos = (0,0)
		self.rot = (0,0,0)

		self.rate = rospy.Rate(10)

	def send_velocity(self, velx, angz):

		self.vel.linear.x = velx
		self.vel.angular.z = angz

		self.pub_vel.publish(self.vel)
		

	def shutdown(self):
		print "shutdown"
		rospy.loginfo("Done!!")

		self.vel.linear.x = 0
		self.vel.angular.z = 0

		self.pub_vel.publish(self.vel)

		rospy.sleep(1)

	def callback_odometry(self,msg):

		self.pos = (msg.pose.pose.position.x,msg.pose.pose.position.y)
		rot_q = msg.pose.pose.orientation
		self.rot = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

def navToPoint(pose,goal,vel):

	dx = goal[0]-pose.pos[0]
	dy = goal[1]-pose.pos[1]

	if abs(dx) < 0.1 and abs(dy) < 0.1:
		stat = True
		return stat
	stat = False
	ang = atan2(dy,dx)

	angZ = (3.8/23.0)*abs(vel[0]-vel[1])*0.5+0.2*0.3
	x = ((38/2.0)*(vel[0]+vel[1])*1e-3)*0.5+0.2*0.3
	if (ang-pose.rot[2])>0.1:
		pose.send_velocity(x,angZ)
		pose.rate.sleep()
	elif (ang-pose.rot[2])<-0.1:
		pose.send_velocity(x,-angZ)
		pose.rate.sleep()
	else:
		pose.send_velocity(x,0)
		pose.rate.sleep()

	return stat

if __name__ == '__main__':

	data = pickle.load(open('/home/teja/tbot/controller/src/control/src/path.pkl','rb'))
	velList = data['vel']
	start = data['start']
	try:

		pubg = pubVel()
		theta = 0
		angZ=0.0
		while not rospy.is_shutdown():
			for vel in velList:
				desX = (vel[2]-start[0])*1e-2
				desY = (start[1]-vel[3])*1e-2
				while True:
					s = navToPoint(pubg,(desX,desY),vel)
					if s:
						break
			break
			
		pubg.shutdown()

	except:
		rospy.loginfo("pubVel node terminated")