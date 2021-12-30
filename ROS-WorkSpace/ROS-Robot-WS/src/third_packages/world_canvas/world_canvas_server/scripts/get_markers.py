#!/usr/bin/env python

import rospy
import yaml
import uuid
import copy
import unique_id
import world_canvas_msgs.msg
import world_canvas_msgs.srv

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from visualization_msgs.msg import Marker, MarkerArray
from world_canvas_utils.serialization import *


def publish(anns, data):
    ar_mk_list = AlvarMarkers()
    marker_list = MarkerArray()    

    marker_id = 1
    for a, d in zip(anns, data):
        
        # AR markers
        object = deserialize_msg(d.data, AlvarMarker)
        ar_mk_list.markers.append(object)
        
        # Visual markers
        marker = Marker()
        marker.id = marker_id
        marker.header = a.pose.header
        marker.type = a.shape
        marker.ns = "ar_mk_obstacles"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.pose = copy.deepcopy(a.pose.pose.pose)
        marker.scale = a.size
        marker.color = a.color

        marker_list.markers.append(marker)

        marker_id = marker_id + 1

    marker_pub = rospy.Publisher('ar_mk_marker',    MarkerArray, latch=True, queue_size=1)
    ar_mk_pub  = rospy.Publisher('ar_mk_pose_list', AlvarMarkers,latch=True, queue_size=1)

    ar_mk_pub.publish(ar_mk_list)
    marker_pub.publish(marker_list)
    
    return


if __name__ == '__main__':
    rospy.init_node('ar_mks_loader')
    world    = rospy.get_param('~world')
    uuids    = rospy.get_param('~uuids', [])
    names    = rospy.get_param('~names', [])
    keywords = rospy.get_param('~keywords', [])
    related  = rospy.get_param('~relationships', [])

    rospy.loginfo("Waiting for get_annotations service...")
    rospy.wait_for_service('get_annotations')

    rospy.loginfo("Loading annotations for world '%s'", world)
    get_anns_srv = rospy.ServiceProxy('get_annotations', world_canvas_msgs.srv.GetAnnotations)
    respAnns = get_anns_srv(world,
                           [unique_id.toMsg(uuid.UUID('urn:uuid:' + id)) for id in uuids],
                            names, ['ar_track_alvar_msgs/AlvarMarker'], keywords,
                           [unique_id.toMsg(uuid.UUID('urn:uuid:' + id)) for id in related])

    if len(respAnns.annotations) > 0:
        rospy.loginfo("Publishing visualization markers for %d retrieved annotations...",
                       len(respAnns.annotations))
    else:
        rospy.loginfo("No annotations found for world '%s' with the given search criteria", world)
        sys.exit()

    rospy.loginfo("Loading data for the %d retrieved annotations", len(respAnns.annotations))
    get_data_srv = rospy.ServiceProxy('get_annotations_data', world_canvas_msgs.srv.GetAnnotationsData)
    respData = get_data_srv([a.data_id for a in respAnns.annotations])

    if len(respData.data) > 0:
        rospy.loginfo("Publishing data for %d retrieved annotations...", len(respData.data))
        publish(respAnns.annotations, respData.data)
    else:
        rospy.logwarn("No data found for the %d retrieved annotations", len(respAnns.annotations))
        
    rospy.loginfo("Done")
    rospy.spin()
