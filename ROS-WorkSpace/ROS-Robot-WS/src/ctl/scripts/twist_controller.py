#!/usr/bin/env python
import importlib
import rospy 
from geometry_msgs.msg import Twist


import keyboard
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o         ^
   j    k    l       < v >
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
        'up':(1,0,0,0),
        'down':(-1,0,0,0),
        'right':(0,0,0,-1),
        'left':(0,0,0,1),
    }
OmnimoveBindings = {
        'i':(1,0,0,0),
        'o':(1,-1,0,0),
        'j':(0,1,0,0),
        'l':(0,-1,0,0),
        'u':(1,1,0,0),
        ',':(-1,0,0,0),
        '.':(-1,-1,0,0),
        'm':(-1,1,0,0),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
        'up':(1,0,0,0),
        'down':(-1,0,0,0),
        'right':(0,-1,0,0),
        'left':(0,1,0,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }
class Controller:
    def __init__(self):
        rospy.init_node("controller",log_level=rospy.INFO)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
        self.speed = rospy.get_param("~speed",0.5)
        self.turn  = rospy.get_param("~turn",1.0)
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.status = 0
        self.omni = False
        self.twist = Twist()
        keyboard.hook(self.input_capture)
        print(msg)
        print(self.vels(self.speed,self.turn))
        keyboard.wait()

    def input_capture(self,input_key):
        if input_key.event_type == 'down':
            if input_key.name == 'shift':
                self.omni = True
            elif input_key.name in moveBindings.keys() and (self.omni==False):
                self.x =  moveBindings[input_key.name][0]
                self.y =  moveBindings[input_key.name][1]
                self.z =  moveBindings[input_key.name][2]
                self.th = moveBindings[input_key.name][3]
                self.twist.linear.x = self.x*self.speed
                self.twist.linear.y = self.y*self.speed
                self.twist.linear.z = self.z*self.speed
                self.twist.angular.x = 0
                self.twist.angular.y = 0
                self.twist.angular.z = self.th*self.turn
                self.cmd_vel_pub.publish(self.twist)
            elif input_key.name in moveBindings.keys() and (self.omni == True):
                self.x =  OmnimoveBindings[input_key.name][0]
                self.y =  OmnimoveBindings[input_key.name][1]
                self.z =  OmnimoveBindings[input_key.name][2]
                self.th = OmnimoveBindings[input_key.name][3]
                self.twist.linear.x = self.x*self.speed
                self.twist.linear.y = self.y*self.speed
                self.twist.linear.z = self.z*self.speed
                self.twist.angular.x = 0
                self.twist.angular.y = 0
                self.twist.angular.z = self.th*self.turn
                self.cmd_vel_pub.publish(self.twist)
            elif input_key.name in speedBindings.keys():
                self.speed = self.speed*speedBindings[input_key.name][0]
                self.turn  = self.turn*speedBindings[input_key.name][1]
                print(self.vels(self.speed,self.turn))
                if(self.status == 14):
                    print(msg)
                self.status = (self.status+1)%15
            else:
                self.x = 0
                self.y = 0
                self.z = 0
                self.th = 0
                self.twist.linear.x = self.x*self.speed
                self.twist.linear.y = self.y*self.speed
                self.twist.linear.z = self.z*self.speed
                self.twist.angular.x = 0
                self.twist.angular.y = 0
                self.twist.angular.z = self.th*self.turn
                self.cmd_vel_pub.publish(self.twist)
        elif input_key.event_type == 'up':
            if input_key.name == 'shift':
                self.omni = False




    def vels(self,speed,turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    def shutdown(self): 
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        cmd_vel_pub.publish(twist)

if __name__=='__main__':
    controller = Controller()


