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
from yocs_msgs.msg import Column, ColumnList
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
        ann.type = 'yocs_msgs/Column'
        for i in range(0, random.randint(0,11)):
            ann.keywords.append('kw'+str(random.randint(1,11)))
        if 'prev_id' in vars():
            ann.relationships.append(prev_id)
        prev_id = ann.id
        ann.shape = 3  #CYLINDER
        ann.color.r = 0.7
        ann.color.g = 0.7
        ann.color.b = 0.7
        ann.color.a = 0.4
        ann.size.x = float(t['radius'])*2
        ann.size.y = float(t['radius'])*2
        ann.size.z = float(t['height'])
        ann.pose.header.frame_id = t['frame_id']
        ann.pose.header.stamp = rospy.Time.now()
        ann.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
        
        # columns are assumed to lay on the floor, so z coordinate is zero;
        # but WCF assumes that the annotation pose is the center of the object
        ann.pose.pose.pose.position.z += ann.size.z/2.0

        anns_list.append(ann)

        object = Column()
        object.name = t['name']
        object.radius = float(t['radius'])
        object.height = float(t['height'])
        object.pose.header.frame_id = t['frame_id']
        object.pose.header.stamp = rospy.Time.now()
        object.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
        data = AnnotationData()
        data.id = ann.data_id
        data.type = ann.type
        data.data = serialize_msg(object)
        
        data_list.append(data)

    return anns_list, data_list

if __name__ == '__main__':
    rospy.init_node('columns_saver')
    world = rospy.get_param('~world')
    file  = rospy.get_param('~file')
    anns, data = read(file)

    rospy.loginfo("Waiting for save_annotations_data service...")
    rospy.wait_for_service('save_annotations_data')
    save_srv = rospy.ServiceProxy('save_annotations_data', world_canvas_msgs.srv.SaveAnnotationsData)

    rospy.loginfo("Saving virtual columns from file '%s' for world '%s'" % (file, world))
    save_srv(anns, data)
    rospy.loginfo("Done")
