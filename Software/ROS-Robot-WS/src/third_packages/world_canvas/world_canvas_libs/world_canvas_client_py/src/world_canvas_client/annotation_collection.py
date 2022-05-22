#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Yujin Robot
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Author: Jorge Santos

import genpy
import rospy
import roslib
import copy
import uuid
import unique_id
import world_canvas_msgs.msg
import world_canvas_msgs.srv

from rospy_message_converter import message_converter
from visualization_msgs.msg import Marker, MarkerArray
from world_canvas_utils.serialization import *
from .exceptions import WCFError


class AnnotationCollection:
    '''
    A collection of annotations and its associated data, initially empty. Annotations
    and data are retrieved from the world canvas server, filtered by a set of selection
    criteria (see constructor parameters).
    The collection can be reloaded with different filter criteria. There're also methods
    to add and delete individual annotations (an "update" method is TODO). But changes are
    only saved to database by calling "save" method. 
    This class can also publish the retrieved annotations and RViz visualization markers,
    mostly for debug purposes.
    '''
    
    def __init__(self, world=None, uuids=[], names=[], types=[], keywords=[], relationships=[], srv_namespace=''):
        '''
        Creates a collection of annotations and its associated data, initially empty.
        Annotations and data are retrieved from the world canvas server, filtered by
        the described parameters.

        @param world:         Annotations in this collection belong to this world.
        @param uuids:         Filter annotations by their uuid.
        @param names:         Filter annotations by their name.
        @param types:         Filter annotations by their type.
        @param keywords:      Filter annotations by their keywords.
        @param relationships: Filter annotations by their relationships.
        @param srv_namespace: World canvas handles can be found under this namespace.
        @raise WCFError:      If something went wrong.
        '''
        if not srv_namespace.endswith('/'):
            self._srv_namespace = srv_namespace + '/'
        else:
            self._srv_namespace = srv_namespace
        self._annotations = list()
        self._annots_data = list()
        self._annots_to_delete = list()
        self._saved = True
        
        if world is not None:
            # Filter parameters provided, so don't wait more to retrieve annotations!
            self.filter_by(world, uuids, names, types, keywords, relationships)

    def filter_by(self, world, uuids=[], names=[], types=[], keywords=[], relationships=[]):
        '''
        Reload annotations collection, filtered by new selection criteria.

        @param world:         Annotations in this collection belong to this world.
        @param uuids:         Filter annotations by their uuid.
        @param names:         Filter annotations by their name.
        @param types:         Filter annotations by their type.
        @param keywords:      Filter annotations by their keywords.
        @param relationships: Filter annotations by their relationships.
        @raise WCFError:      If something went wrong.
        '''
        rospy.loginfo("Getting annotations for world '%s' and additional filter criteria", world)
        get_anns_srv = self._get_service_handle('get_annotations', world_canvas_msgs.srv.GetAnnotations)

        ids = [unique_id.toMsg(uuid.UUID('urn:uuid:' + id)) for id in uuids]
        relationships = [unique_id.toMsg(uuid.UUID('urn:uuid:' + id)) for id in relationships]
        response = get_anns_srv(world, ids, names, types, keywords, relationships)

        if response.result:
            if len(response.annotations) > 0:
                rospy.loginfo("%d annotations found", len(response.annotations))
                self._annotations = response.annotations
            else:
                rospy.loginfo("No annotations found for world '%s' with the given search criteria", world)
                del self._annotations[:]
            
            # All non-saved information gets invalidated after reload the collection
            del self._annots_data[:]
            del self._annots_to_delete[:]
            self._saved = True
        else:
            message = "Server reported an error: %s" % response.message
            rospy.logerr(message)
            raise WCFError(message)

    def get_annotations(self, name=None, type=None):
        '''
        Return the currently loaded annotations, optionally restricted to the given name and/or type.

        @param name: Get only annotation(s) with the given name. Can be more than one, as we don't
        enforce uniqueness on annotation names.
        @param type: Get only annotation(s) with the given type.
        @returns Currently loaded annotations matching name and type, or empty string if none.
        '''
        ret = []
        if len(self._annotations) == 0:
            message = "No annotations retrieved. Nothing to return!"
            rospy.logwarn(message)
        else:
            # copying before return to prevent unexpected data curruption
            if name:
                name_filtered = [a for a in self._annotations if a.name == name]
            else:
                name_filtered = [a for a in self._annotations]

            if type:
                ret = [copy.deepcopy(a) for a in name_filtered if a.type == type]
            else:
                ret = [copy.deepcopy(a) for a in name_filtered]
        return ret

    def load_data(self):
        '''
        Load associated data for the current annotations collection.

        @returns Number of annotations retrieved.
        @raise WCFError: If something went wrong.
        '''
        if len(self._annotations) == 0:
            message = "No annotations retrieved. Nothing to load!"
            rospy.logwarn(message)
            return 0

        get_data_srv = self._get_service_handle('get_annotations_data', world_canvas_msgs.srv.GetAnnotationsData)
        rospy.loginfo("Loading data for the %d retrieved annotations", len(self._annotations))
        response = get_data_srv([a.data_id for a in self._annotations])

        if response.result:
            if len(response.data) > 0:
                rospy.loginfo("%d annotations data retrieved", len(response.data))
                self._annots_data = response.data
            else:
                rospy.logwarn("No data found for the %d retrieved annotations", len(self._annotations))
                del self._annots_data[:]
        else:
            message = "Server reported an error: %s" % response.message
            rospy.logerr(message)
            raise WCFError(message)

        return len(response.data)

    def get_data(self, annotation):
        '''
        Get the associated data (ROS message) of a given annotation.

        @returns The ROS message associated to the given annotation or None if it was not found.
        @raise WCFError: If something else went wrong.
        '''
        for d in self._annots_data:
            if d.id == annotation.data_id:
                # Keep the class of the messages to be published; we need it later when deserializing them
                msg_class = roslib.message.get_message_class(d.type)
                if msg_class is None:
                    # This could happen if the message type is wrong or not known for this node (i.e. its
                    # package is not on ROS_PACKAGE_PATH). Both cases are really weird in the client side.
                    message = "Data type '%s' definition not found" % d.type
                    rospy.logerr(message)
                    raise WCFError(message)
                try:
                    object = deserialize_msg(d.data, msg_class)
                except SerializationError as e:
                    message = "Deserialization failed: %s" % str(e)
                    rospy.logerr(message)
                    raise WCFError(message)
                return object
        rospy.logwarn("Data uuid not found: " + unique_id.toHexString(annotation.data_id))
        return None

    def get_data_of_type(self, type):
        '''
        Get all associated data (ROS messages) of a given type.

        @returns The list of ROS messages of the given type.
        '''
        result = list()

        for d in self._annots_data:
            try:
                object = deserialize_msg(d.data, type)
            except SerializationError as e:
                # rospy.logerr("Deserialization failed: %s' % str(e))
                # TODO/WARN: this is ok, as I'm assuming that deserialize will always fail with messages of
                # different types, so I use it as filter. It works by now BUT I'm not 100% sure about that!
                # The TODO is that I should organize both annotations and data in a unified list. Once I have
                # it, make a version of this method with default param type=None and use annotation.type to
                # deserialize each object
                continue

            result.append(object)

        return result;

    def publish_markers(self, topic):
        '''
        Publish RViz visualization markers for the current collection of annotations.

        @param topic: Where we must publish annotations markers.
        '''
        if len(self._annotations) == 0:
            messages = "No annotations retrieved. Nothing to publish!"
            rospy.logwarn(messages)
            return

        # Advertise a topic for retrieved annotations' visualization markers
        markers_pub = rospy.Publisher(topic, MarkerArray, latch=True, queue_size=5)

        # Process retrieved data to build markers lists
        markers_list = MarkerArray()

        marker_id = 1
        for a in self._annotations:
            marker = Marker()
            marker.id = marker_id
            marker.header = a.pose.header
            marker.type = a.shape
            marker.ns = a.type
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration.from_sec(0)
            marker.pose = copy.deepcopy(a.pose.pose.pose)
            marker.scale = a.size
            marker.color = a.color

            markers_list.markers.append(marker)

            label = copy.deepcopy(marker)
            label.id = label.id + 1000000 # marker id must be unique
            label.type = Marker.TEXT_VIEW_FACING
            label.text = a.name + ' [' + a.type + ']'
            label.pose.position.z = label.pose.position.z + label.scale.z/2.0 + 0.12 # just above the visual
            label.scale.x = label.scale.y = label.scale.z = 0.3
            label.color.a = 1.0

            markers_list.markers.append(label)

            marker_id = marker_id + 1


        markers_pub.publish(markers_list)

    def publish(self, topic_name, topic_type=None, by_server=False, as_list=False, list_attribute=None):
        '''
        Publish the current collection of annotations, by this client or by the server.
        As we use just one topic, all annotations must be of the same type (function will return
        with error otherwise).

        @param topic_name: Where we must publish annotations data.
        @param topic_type: The message type to publish annotations data.
                           Mandatory if as_list is true; ignored otherwise.
        @param by_server:  Request the server to publish the annotations instead of this client.
        @param as_list:    If true, annotations will be packed in a list before publishing,
                           so topic_type must be an array of currently loaded annotations.
        @raise WCFError:   If something went wrong.
        '''
        if len(self._annotations) == 0:
            message = "No annotations retrieved. Nothing to publish!"
            rospy.logwarn(message)
            return

        if by_server:
            # Request server to publish the annotations previously retrieved
            pub_data_srv = self._get_service_handle('pub_annotations_data', world_canvas_msgs.srv.PubAnnotationsData)
            rospy.loginfo("Requesting server to publish annotations")
            response = pub_data_srv([a.data_id for a in self._annotations], topic_name, topic_type, as_list)
            if not response.result:
                message = "Server reported an error: %s" % response.message
                rospy.logerr(message)
                raise WCFError(message)
        else:
            # Take annotations message type and verify that it's the same within the collection,
            # as we will publish all the elements with the same topic (as a list or one by one)
            for a in self._annotations:
                if 'msg_type' not in locals():
                    msg_type = a.type
                elif msg_type != a.type:
                    message = "Cannot publish annotations of different types (%s, %s)" % (msg_type, a.type)
                    rospy.logerr(message)
                    raise WCFError(message)

            if 'msg_type' not in locals():
                message = "Annotations message type not found? impossible! (we already checked at method start)"
                rospy.logerr(message)
                raise WCFError(message)

            # Keep the class of the messages to be published; we need it later when deserializing them
            msg_class = roslib.message.get_message_class(msg_type)
            if msg_class is None:
                # This could happen if the message type is wrong or not known for this node (i.e. its
                # package is not on ROS_PACKAGE_PATH). Both cases are really weird in the client side.
                message = "Topic type '%s' definition not found" % topic_type
                rospy.logerr(message)
                raise WCFError(message)

            # Advertise a topic with message type topic_type if we will publish results as a list (note that we
            # ignore list's type) or use current annotations type otherwise (we have verified that it's unique)
            if as_list:
                if topic_type is None:
                    message ="Topic type argument is mandatory if as_list is true"
                    rospy.logerr(message)
                    raise WCFError(message)
                topic_class = roslib.message.get_message_class(topic_type)
                if topic_class is None:
                    # Same comment as in "msg_class is None" applies here
                    message = "Topic type '%s' definition not found" % topic_type
                    rospy.logerr(message)
                    raise WCFError(message)
            else:
                topic_class = msg_class

            # Advertise a topic to publish retrieved annotations
            objects_pub = rospy.Publisher(topic_name, topic_class, latch=True, queue_size=5)

            # Process retrieved data to build annotations lists
            objects_list = self.get_data_of_type(msg_class)

            # Publish resulting list
            if as_list:
                
                if list_attribute:
                    try:
                        msg = topic_class() 
                        setattr(msg, list_attribute, objects_list)
                    except AttributeError as e: 
                        message = "List attribute caused error[%s]" % str(e)
                        rospy.logerr(message)
                        raise WCFError(message)
                else:
                    msg = topic_class(objects_list)
                objects_pub.publish(msg)
            else:
                # if as_list is false, publish objects one by one
                for object in objects_list:
                    objects_pub.publish(object)

    def add(self, annotation, msg=None, gen_uuid=True):
        '''
        Add a new annotation with a new associated data or for an existing data.

        @param annotation: The new annotation.
        @param msg:        Its associated data. If None, we assume that we are adding an annotation to existing data.
        @param gen_uuid:   Generate an unique id for the new annotation or use the received one.
        @raise WCFError:   If something went wrong.
        '''
        if gen_uuid:
            annotation.id = unique_id.toMsg(unique_id.fromRandom())
        else:
            for a in self._annotations:
                if a.id == annotation.id:
                    message = "Duplicated annotation with uuid '%s'" % unique_id.toHexString(annotation.id)
                    rospy.logerr(message)
                    raise WCFError(message)

        if msg is None:
            # Msg not provided, so we assume that we are adding an annotation to existing data; find it by its id
            msg_found = False
            for d in self._annots_data:
                if d.id == annotation.data_id:
                    rospy.logdebug("Annotation data with uuid '%s' found", unique_id.toHexString(annotation.data_id))
                    msg_found = True
                    break

            if not msg_found:
                message = "Annotation data with uuid '%s' not found" % unique_id.toHexString(annotation.data_id)
                rospy.logerr(message)
                raise WCFError(message)
        else:
            # Annotation comes with its data; create a common unique id to link both
            annotation.data_id = unique_id.toMsg(unique_id.fromRandom())
            annot_data = world_canvas_msgs.msg.AnnotationData()
            annot_data.id = annotation.data_id
            annot_data.type = annotation.type
            annot_data.data = serialize_msg(msg)
            self._annots_data.append(annot_data)

        self._annotations.append(annotation)
        self._saved = False


    def update(self, annotation, msg=None):
        '''
        Update an existing annotation and optionally its associated data.

        @param annotation: The modified annotation.
        @param msg:        Its associated data. If None, just the annotation is updated.
        @raise WCFError:   If something went wrong.
        '''
        found = False
        for i, a in enumerate(self._annotations):
            if a.id == annotation.id:
                self._annotations[i] = annotation
                found = True
                break

        if not found:
            message = "Annotation with uuid '%s' not found" % unique_id.toHexString(annotation.id)
            rospy.logerr(message)
            raise WCFError(message)

        if msg is None:
            return
        
        found = False
        for i, d in enumerate(self._annots_data):
            if d.id == annotation.data_id:
                rospy.logdebug("Annotation data with uuid '%s' found", unique_id.toHexString(annotation.data_id))
                self._annots_data[i].type = annotation.type
                self._annots_data[i].data = serialize_msg(msg)
                found = True
                break

        if not found:
            message = "Annotation data with uuid '%s' not found" % unique_id.toHexString(annotation.data_id)
            rospy.logerr(message)
            raise WCFError(message)

        self._saved = False

    def remove(self, uuid):
        '''
        Delete an annotation with its associated data.
        WARN/TODO: we are ignoring the case of N annotations - 1 data!

        @param uuid: The uuid of the annotation to delete.
        @returns True if successfully removed, False if the annotation was not found.
        @raise WCFError: If something else went wrong.
        '''
        for a in self._annotations:
            if a.id == uuid:
                rospy.logdebug("Annotation '%s' found", unique_id.toHexString(uuid))
                ann_to_delete = a
                break

        if 'ann_to_delete' not in locals():
            rospy.logwarn("Annotation '%s' not found", unique_id.toHexString(uuid))
            return False

        for d in self._annots_data:
            if d.id == a.data_id:
                rospy.logdebug("Annotation data '%s' found", unique_id.toHexString(d.id))
                data_to_delete = d
                break

        if 'data_to_delete' not in locals():
            message = "No data found for annotation '%s' (data uuid is '%s')" \
                    % (unique_id.toHexString(uuid), unique_id.toHexString(ann_to_delete.data_id))
            rospy.logerr(message)
            raise WCFError(message)

        rospy.logdebug("Removed annotation with uuid '%s'", unique_id.toHexString(ann_to_delete.id))
        rospy.logdebug("Removed annot. data with uuid '%s'", unique_id.toHexString(data_to_delete.id))
        self._annotations.remove(ann_to_delete)
        self._annots_data.remove(data_to_delete)
        
        self._annots_to_delete.append(ann_to_delete)
        self._saved = False

        return True

    def save(self):
        '''
        Save to database current annotations list with their associated data. Also remove from database
        the annotations doomed by delete method, if any.
        WARN/TODO: we are ignoring the case of N annotations - 1 data!

        @raise WCFError: If something went wrong.
        '''
        rospy.loginfo("Requesting server to save annotations")
        annotations = []
        annots_data = []

        # This brittle saving procedure requires parallelly ordered annotations and data vectors
        # As this don't need to be the case, we must short them; but we need a better saving procedure (TODO)
        for a in self._annotations:
            for d in self._annots_data:
                if a.data_id == d.id:
                    rospy.logdebug("Add annotation for saving with uuid '%s'", unique_id.toHexString(a.id))
                    rospy.logdebug("Add annot. data for saving with uuid '%s'", unique_id.toHexString(d.id))
                    annotations.append(a)
                    annots_data.append(d)
                    break

        # Do at least a rudimentary check
        if not (len(self._annotations) == len(self._annots_data) == len(annotations) == len(annots_data)):
            message = "Incoherent annotation and data sizes: %d != %d != %d != %d" \
                    % (len(self._annotations), len(self._annots_data), len(annotations), len(annots_data))
            rospy.logerr(message)
            raise WCFError(message)

        # Request server to save current annotations list, with its data
        save_data_srv = self._get_service_handle('save_annotations_data', world_canvas_msgs.srv.SaveAnnotationsData)
        rospy.loginfo("Requesting server to save annotations")
        response = save_data_srv(annotations, annots_data)
        if not response.result:
            message = "Server reported an error: %s" % response.message
            rospy.logerr(message)
            raise WCFError(message)
        
        # We must also remove from database the annotations doomed by delete method, if any
        if len(self._annots_to_delete) > 0:
            del_anns_srv = self._get_service_handle('delete_annotations', world_canvas_msgs.srv.DeleteAnnotations)
            rospy.loginfo("Requesting server to delete %d doomed annotations" % len(self._annots_to_delete))
            response = del_anns_srv(self._annots_to_delete)
            if not response.result:
                message = "Server reported an error: %s" % response.message
                rospy.logerr(message)
                raise WCFError(message)

        self._saved = True

    def is_saved(self):
        return self._saved

    def get_world_list(self):
        '''
        TODO: temporally here until we create a WorldCollection class, as on C++ library.
        
        @returns Currently available world list in the world canvas server
        '''
        get_world_list_srv = self._get_service_handle('list_worlds', world_canvas_msgs.srv.ListWorlds)
        response = get_world_list_srv()
        
        return response.names

    def _get_service_handle(self, service_name, service_type, timeout=5.0):
        '''
        Create a service client and wait until the service is available.

        @param service_name: ROS service name to get, without namespace.
        @param service_type: ROS service type.
        @param timeout: Timeout to wait for the service to come up.

        @returns: Service handle.
        @rtypes: rospy.ServiceProxy.

        @raise  WCFError: If specified timeout is exceeded or shutdown interrupts wait.
        '''
        try:
            rospy.loginfo("Waiting for '%s' service..." % str(service_name))
            rospy.wait_for_service(self._srv_namespace + service_name, timeout)
            srv = rospy.ServiceProxy(self._srv_namespace + service_name, service_type)
            return srv
        except rospy.exceptions.ROSInterruptException as e:
            raise WCFError("Wait interrupted by shutdown: %s" % str(e))
        except rospy.exceptions.ROSException as e:
            raise WCFError("%d seconds elapsed: %s" % (timeout, str(e)))
