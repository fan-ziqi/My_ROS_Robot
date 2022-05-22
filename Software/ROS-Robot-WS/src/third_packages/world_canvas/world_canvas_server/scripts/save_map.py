#!/usr/bin/env python

import genpy
import rospy
import yaml
import uuid
import unique_id
import world_canvas_msgs.msg
import world_canvas_msgs.srv

from nav_msgs.msg import *
from rospy_message_converter import message_converter
from world_canvas_msgs.msg import Annotation, AnnotationData
from world_canvas_utils.serialization import *


def read(file):
    '''
    Parse a yaml file containing a single map message
    @param file Target file path
    '''
    yaml_data = None 
    with open(file) as f:
       yaml_data = yaml.load(f)
      
    ann = Annotation()
    ann.timestamp = rospy.Time.now()
    ann.data_id = unique_id.toMsg(unique_id.fromRandom())
    ann.id = unique_id.toMsg(unique_id.fromRandom())
    ann.world = world
    ann.name = map_name
    ann.type = 'nav_msgs/OccupancyGrid'
    ann.keywords.append(map_name)
    ann.shape = 1 # CUBE
    ann.color.r = 0.2
    ann.color.g = 0.2
    ann.color.b = 0.2
    ann.color.a = 0.01
    ann.size.x = yaml_data['info']['width'] * yaml_data['info']['resolution']
    ann.size.y = yaml_data['info']['height'] * yaml_data['info']['resolution']
    ann.size.z = 0.000001
    ann.pose.header.frame_id = '/map'
    ann.pose.header.stamp = rospy.Time.now()
    ann.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',
                                                                              yaml_data['info']['origin'])

    object = OccupancyGrid()
    genpy.message.fill_message_args(object, yaml_data)
    map = AnnotationData()
    map.id = ann.data_id
    map.type = ann.type
    map.data = serialize_msg(object)
    
    return [ann], [map]  # return as lists, as is what expects save_annotations_data service

if __name__ == '__main__':
    rospy.init_node('map_saver')
    file     = rospy.get_param('~file')
    world    = rospy.get_param('~world')
    map_name = rospy.get_param('~map_name')
    ann, map = read(file)

    rospy.loginfo("Waiting for save_annotations_data service...")
    rospy.wait_for_service('save_annotations_data')
    save_srv = rospy.ServiceProxy('save_annotations_data', world_canvas_msgs.srv.SaveAnnotationsData)

    rospy.loginfo("Saving map from file '%s' for world '%s'" % (file, world))
    save_srv(ann, map)
    rospy.loginfo("Done")
