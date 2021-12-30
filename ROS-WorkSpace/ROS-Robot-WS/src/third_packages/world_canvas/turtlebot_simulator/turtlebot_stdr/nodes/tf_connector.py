#!/usr/bin/env python
__author__ = 'mehdi tlili'
import rospy
from tf2_msgs.msg import TFMessage
import tf

class Remapper(object):

    def __init__(self):
        self.br = tf.TransformBroadcaster()
        rospy.Subscriber("/tf", TFMessage, self.tf_remapper)

    def tf_remapper(self, msg):

        if msg.transforms[0].header.frame_id == "/robot0":
            self.br.sendTransform((0, 0, 0),
                                  tf.transformations.quaternion_from_euler(0, 0, 0),
                                  rospy.Time.now(),
                                  "base_footprint",
                                  "robot0")


if __name__ == '__main__':
    rospy.init_node('remapper_nav')
    remapper = Remapper()
    rospy.spin()
