#!/usr/bin/env python

import rospy
import yaml
import random
import uuid
import unique_id
import world_canvas_msgs.msg
import world_canvas_msgs.srv

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from world_canvas_msgs.msg import Annotation, AnnotationData
from world_canvas_utils.serialization import *


def read(file):
    yaml_data = None 
    with open(file) as f:
       yaml_data = yaml.load(f)

    anns_list = []
    data_list = []

    for t in yaml_data:
        ann = Annotation()
        ann.timestamp = rospy.Time.now()
        ann.data_id = unique_id.toMsg(unique_id.fromRandom())
        ann.id = unique_id.toMsg(unique_id.fromRandom())
        ann.world = world
        ann.name = t['name']
        ann.type = 'ar_track_alvar_msgs/AlvarMarker'
        for i in range(0, random.randint(0,11)):
            ann.keywords.append('kw'+str(random.randint(1,11)))
        # if 'prev_id' in vars():
        #     ann.relationships.append(prev_id)
        # prev_id = ann.id
        ann.shape = 1 # CUBE
        ann.color.r = 1.0
        ann.color.g = 1.0
        ann.color.b = 1.0
        ann.color.a = 1.0
        ann.size.x = 0.18
        ann.size.y = 0.18
        ann.size.z = 0.01
        ann.pose.header.frame_id = t['frame_id']
        ann.pose.header.stamp = rospy.Time.now()
        ann.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])

        anns_list.append(ann)
        
        object = AlvarMarker()
        object.id = t['id']
        object.confidence = t['confidence']
        object.pose.header.frame_id = t['frame_id']
        object.pose.header.stamp = rospy.Time.now()
        object.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
        data = AnnotationData()
        data.id = ann.data_id
        data.type = ann.type
        data.data = serialize_msg(object)
        
        data_list.append(data)

    return anns_list, data_list



if __name__ == '__main__':
    rospy.init_node('markers_saver')
    world = rospy.get_param('~world')
    file  = rospy.get_param('~file')
    anns, data = read(file)

    rospy.loginfo("Waiting for save_annotations_data service...")
    rospy.wait_for_service('save_annotations_data')
    save_srv = rospy.ServiceProxy('save_annotations_data', world_canvas_msgs.srv.SaveAnnotationsData)

    rospy.loginfo("Saving AR markers from file '%s' for world '%s'" % (file, world))
    save_srv(anns, data)
    rospy.loginfo("Done")
