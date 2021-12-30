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

class MultiGoalListen:
    
    def __init__(self):
        rospy.init_node('MultiGoalListen', anonymous = False)
        rospy.on_shutdown(self.shutdown)
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Waiting for move_base action server...')
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo('Connected to move base server')
        rospy.loginfo('Starting MultiGoalListen')
        rospy.Subscriber('/robot_pose', Pose, self.robot_pose_callback)
        self.r = redis.Redis(host = '127.0.0.1', port = 6379, db = 0)
        attr_dict = {
            'mode': 'order',
            'loopWay': 'auto',
            'isNavNext': 0,
            'currentState': 'stopped',
            'goalQueue': 'GoalQueueA',
            'priorGoalQueue': 'GoalQueueB',
            'currentQueue': '',
            'currentGoal': '',
            'successNum': '0',
            'failedNum': '0',
            'intervalTime': '3' }
        self.r.hmset('GoalState', attr_dict)
        goal = MoveBaseGoal()
        quaternion = Quaternion()
        while not rospy.is_shutdown():
            mymode = self.r.hget('GoalState', 'mode')
            priorgoalQueue = self.r.hget('GoalState', 'priorGoalQueue')
            goalQueue = self.r.hget('GoalState', 'goalQueue')
            loopWay = self.r.hget('GoalState', 'loopWay')
            isNavNext = self.r.hget('GoalState', 'isNavNext')
            if priorgoalQueue != None and self.r.llen(priorgoalQueue) > 0:
                mygoalQueue = priorgoalQueue
            else:
                mygoalQueue = goalQueue
            mycurrentState = self.r.hget('GoalState', 'currentState')
            if mycurrentState != 'running':
                rospy.sleep(1)
                continue
            if loopWay == 'manual':
                if int(isNavNext) == 1:
                    self.r.hset('GoalState', 'isNavNext', 0)
                else:
                    rospy.sleep(1)
            self.r.hset('GoalState', 'currentQueue', mygoalQueue)
            mygoal = self.r.lpop(mygoalQueue)
            if mygoalQueue == goalQueue and mymode == 'loop':
                self.r.rpush(mygoalQueue, mygoal)
                rospy.loginfo('rpush:' + mygoalQueue + ' ' + mygoal)
            if mygoal != None:
                self.r.hset('GoalState', 'currentGoal', mygoal)
                mygoalVal = self.r.hget('GoalAliaseSet', mygoal)
                if mygoalVal != None:
                    (x, y, z, qx, qy, qz, qw) = str.split(mygoalVal, '_')
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    quaternion.x = float(qx)
                    quaternion.y = float(qy)
                    quaternion.z = float(qz)
                    quaternion.w = float(qw)
                    goal.target_pose.pose.position.x = float(x)
                    goal.target_pose.pose.position.y = float(y)
                    goal.target_pose.pose.position.z = float(z)
                    goal.target_pose.pose.orientation = quaternion
                    self.move_base.send_goal(goal)
                    finished_within_time = self.move_base.wait_for_result(rospy.Duration(600))
                    if not finished_within_time:
                        self.move_base.cancel_goal()
                        rospy.logwarn('Timed out achieving goal')
                        self.r.hincrby('GoalState', 'failedNum')
                    else:
                        state = self.move_base.get_state()
                        if state == GoalStatus.SUCCEEDED:
                            rospy.loginfo('Goal succeeded!')
                            self.r.hincrby('GoalState', 'successNum')
                        else:
                            rospy.logwarn('Goal failed, state:' + str(state))
                            self.r.hincrby('GoalState', 'failedNum')
                
            else:
                rospy.loginfo('all Goales have been finished')
                self.r.hset('GoalState', 'currentState', 'stopped')
            rospy.sleep(int(self.r.hget('GoalState', 'intervalTime')))

    
    def robot_pose_callback(self, msg):
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        attr_dict = {
            'x': '%f' % x,
            'y': '%f' % y,
            'z': '%f' % z,
            'qx': '%f' % qx,
            'qy': '%f' % qy,
            'qz': '%f' % qz,
            'qw': '%f' % qw }
        self.r.hmset('CurrentRobotPose', attr_dict)

    
    def shutdown(self):
        rospy.loginfo('Stopping the robot...')
        self.move_base.cancel_goal()
        rospy.sleep(2)

if __name__ == '__main__':
    MultiGoalListen()
