#!/usr/bin/env python
# encoding: utf-8
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
import redis
import datetime
import os
import time

class CancelGoalListen:
    
    def __init__(self):
        rospy.init_node('cancel_goal_listen', anonymous = False)
        rospy.on_shutdown(self.shutdown)
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Waiting for move_base action server...')
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo('Connected to move base server')
        rospy.loginfo('Starting cancel_goal_listen')
        self.cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size = 10)
        self.r = redis.Redis(host = '127.0.0.1', port = 6379, db = 0)
        self.lastState = 'stopped'
        while not rospy.is_shutdown():
            mycurrentState = self.r.hget('GoalState', 'currentState')
            if mycurrentState == 'stopped' and self.lastState == 'running':
                mycurrentGoal = self.r.hget('GoalState', 'currentGoal')
                mygoalQueue = self.r.hget('GoalState', 'goalQueue')
                mymode = self.r.hget('GoalState', 'mode')
                if mymode == 'loop':
                    self.r.rpop(mygoalQueue)
                self.r.lpush(mygoalQueue, mycurrentGoal)
                rospy.loginfo('cancel current goal.')
                goalId = GoalID()
                self.cancel_pub.publish(goalId)
            self.lastState = mycurrentState
            rospy.sleep(1)

    
    def shutdown(self):
        rospy.loginfo('Stopping the robot...')
        self.move_base.cancel_goal()
        rospy.sleep(2)

if __name__ == '__main__':
    CancelGoalListen()

