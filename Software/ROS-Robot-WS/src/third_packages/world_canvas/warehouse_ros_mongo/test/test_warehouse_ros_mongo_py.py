#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Bhaskara Marthi
#
# 1

## %Tag(PYTHON_CLIENT)%

import roslib; roslib.load_manifest('warehouse_ros')
import rospy
import warehouse_ros as wr
import unittest
import math
import geometry_msgs.msg as gm

def make_pose(x, y, th):
    return gm.Pose(gm.Point(x, y, 0),
            gm.Quaternion(0, 0, math.sin(th/2), math.cos(th/2)))
    
def make_metadata(p, str):
    return {'x': p.position.x, 'y': p.position.y, 'name': str}

def to_tuple(p):
    return (p.position.x, p.position.y, p.position.z, p.orientation.x,
            p.orientation.y, p.orientation.z, p.orientation.w)

def eq_poses(p1, p2):
    return to_tuple(p1)==to_tuple(p2)
            

class TestWarehouseRosMongoPy(unittest.TestCase):

    def test_basic(self):

        # Set up collection
        coll = wr.MessageCollection("my_db", "poses", gm.Pose)
        p1 = make_pose(24, 42, 0)
        p2 = make_pose(10, 532, 3)
        p3 = make_pose(53, 22, 5)
        p4 = make_pose(22, -5, 33)

        # Insert pose objects with accompanying string metadata
        coll.insert(p1, make_metadata(p1, "bar"))
        coll.insert(p2, make_metadata(p2, "baz"))
        coll.insert(p3, make_metadata(p3, "qux"))
        coll.insert(p1, make_metadata(p1, "oof"))
        coll.insert(p4, make_metadata(p4, "ooof"))

        # Query poses s.t x < 40, y > ), in descending order of name
        results = coll.query({'x': {'$lt': 40}, 'y': {'$gt': 0}},\
                sort_by='name', ascending=False)

        # Turn list of pairs into pair of lists
        poses, metadata = zip(*list(results))
        
        self.assertEqual(len(poses), 3)
        self.assertTrue(eq_poses(p1, poses[0]))
        self.assertTrue(eq_poses(p2, poses[1]))
        self.assertTrue(eq_poses(p1, poses[2]))

        self.assertEqual(metadata[0]['name'], 'oof')
        self.assertEqual(metadata[1]['name'], 'baz')
        self.assertEqual(metadata[2]['name'], 'bar')

        # Update some messages
        coll.update(metadata[0], metadata={'name': 'bat'})
        coll.update(metadata[2], msg=p2)
        res2 = coll.query({'y': {'$gt': 40}}, sort_by='name')
        poses, metadata = zip(*list(res2))

        self.assertEqual(metadata[0]['name'], 'bar')
        self.assertEqual(metadata[1]['name'], 'bat')
        self.assertEqual(metadata[2]['name'], 'baz')
        self.assertTrue(eq_poses(p2, poses[0]))
        self.assertTrue(eq_poses(p1, poses[1]))
        self.assertTrue(eq_poses(p2, poses[2]))
        

        # Remove entries s.t. y<30
        self.assertEqual(5, coll.count())
        self.assertEqual(2, coll.remove({'y': {'$lt': 30}}))
        self.assertEqual(3, coll.count())

        # Test find_one
        self.assertTrue(coll.find_one({'y': {'$gt': 30}}))
        self.assertFalse(coll.find_one({'y': {'$lt': 30}}))
        

if __name__ == "__main__":
    rospy.init_node('test_warehouse_ros_mongo_py')
    import rostest
    rostest.rosrun('warehouse_ros_mongo', 'test_warehouse_ros_mongo_py', TestWarehouseRosMongoPy)

## %EndTag(PYTHON_CLIENT)%
