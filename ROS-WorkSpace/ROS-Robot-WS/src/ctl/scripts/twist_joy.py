#!/usr/bin/env python
import importlib

import rospy
from sensor_msgs.msg import Joy
import time
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String as StringMsg

class JoyTeleop:
	def __init__(self):
		self.x_speed_scale = rospy.get_param('~x_speed_scale')
		self.y_speed_scale = rospy.get_param('~y_speed_scale')
		self.w_speed_scale = rospy.get_param('~w_speed_scale')
		self.velocity = Twist()
		self.rate = rospy.Rate(20)
                self.active = 0
		
		self.cmdVelPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 3)
		self.joySubscriber = rospy.Subscriber('joy', Joy, self.buttonCallback)
		self.loop()
	def buttonCallback(self, joy_data):
		if(joy_data.buttons[4] == 1):
			self.velocity.linear.x = self.x_speed_scale * joy_data.axes[4]
			self.velocity.linear.y = self.y_speed_scale * joy_data.axes[3]
			self.velocity.angular.z = self.w_speed_scale * joy_data.axes[0]
                        self.active = 1
		else:
			self.velocity.linear = Vector3(0.,0.,0.)
			self.velocity.angular = Vector3(0.,0.,0.)
                        self.active = 0
			self.cmdVelPublisher.publish(self.velocity)

                            
	def loop(self):
		while not rospy.is_shutdown():
                        if(self.active == 1):
			    self.cmdVelPublisher.publish(self.velocity)
			self.rate.sleep()
			

	
if __name__ == '__main__':
	rospy.init_node('joy_teleop')
	joy = JoyTeleop()
	try:
		rospy.spin()
	except	rospy.ROSInterruptException:
		print('exception')
			
	
