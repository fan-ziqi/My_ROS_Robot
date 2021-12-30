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

import roslib.message
import rospy
import unique_id

import warehouse_ros_mongo as wr

from map_manager import *
from yaml_database import *

from world_canvas_msgs.msg import *
from world_canvas_msgs.srv import *
from world_canvas_utils.serialization import *


class AnnotationsServer:
    
    ##########################################################################
    # Initialization
    ##########################################################################

    def __init__(self):
        # Set up collections: for every annotation, we store an AnnotationData message plus 1 or
        # more Annotation messages, the first containing the an id and a serialized representation
        # of a message of type annotation.type (data field)
        self.world_collection = \
            wr.MessageCollection("world_canvas", "worlds", WorldCanvas)
        self.world_collection.ensure_index("id", unique=True)

        self.anns_collection = \
            wr.MessageCollection("world_canvas", "annotations", Annotation)
        self.anns_collection.ensure_index("id", unique=True)

        self.data_collection = \
            wr.MessageCollection("world_canvas", "annotations_data", AnnotationData)
        self.data_collection.ensure_index("id", unique=True)
        
        # Set up services
        self.get_anns_srv = \
            rospy.Service('get_annotations',      GetAnnotations,     self.get_annotations)
        self.get_data_srv = \
            rospy.Service('get_annotations_data', GetAnnotationsData, self.get_annotations_data)
        self.pub_data_srv = \
            rospy.Service('pub_annotations_data', PubAnnotationsData, self.pub_annotations_data)

        self.del_anns_srv = \
            rospy.Service('delete_annotations',    DeleteAnnotations,   self.delete_annotations)
        self.save_data_srv = \
            rospy.Service('save_annotations_data', SaveAnnotationsData, self.save_annotations_data)

        self.list_worlds_srv = \
            rospy.Service('list_worlds',      ListWorlds,      self.list_worlds)

        self.set_keyword_srv = \
            rospy.Service('set_keyword',      SetKeyword,      self.set_keyword)
        self.set_related_srv = \
            rospy.Service('set_relationship', SetRelationship, self.set_relationship)

        self.reset_database_srv = \
            rospy.Service('reset_database', ResetDatabase, self.reset_database)

        # Configure services for import from/export to YAML file
        self.yaml_db = YAMLDatabase(self.anns_collection, self.data_collection)

        # Configure services for backward compatible map management
        if rospy.get_param('~start_map_manager', False):
            self.map_mng = MapManager()
        
        rospy.loginfo("Annotations server : initialized.")


    ##########################################################################
    # Services callbacks
    ##########################################################################

    def get_annotations(self, request):

        response = GetAnnotationsResponse()
        
        # Compose query concatenating filter criteria in an '$and' operator
        # Except world, all criteria are lists: operator '$in' makes a N to N matching
        # Empty fields are ignored
        query = {'$and':[]}
        query['$and'].append({'world': {'$in': [request.world]}})
        if len(request.ids) > 0:
            query['$and'].append({'id': {'$in': [unique_id.toHexString(id) for id in request.ids]}})
        if len(request.names) > 0:
            query['$and'].append({'name': {'$in': request.names}})
        if len(request.types) > 0:
            query['$and'].append({'type': {'$in': request.types}})
        if len(request.keywords) > 0:
            query['$and'].append({'keywords': {'$in': request.keywords}})
        if len(request.relationships) > 0:
            query['$and'].append({'relationships': {'$in': [unique_id.toHexString(r) for r in request.relationships]}})

        # Execute the query and retrieve results
        rospy.logdebug("Find annotations with query %s" % query)
        matching_anns = self.anns_collection.query(query)            

        i = 0
        while True:
            try:
                response.annotations.append(matching_anns.next()[0])
                i += 1
            except StopIteration:
                if (i == 0):
                    rospy.loginfo("No annotations found")
                    return self.service_success(response)  # we don't consider this an error
                break
    
    
#         if (len(matching_anns) != len(matching_data)):
#             # we consider this an error by now, as we assume a 1 to 1 relationship;
#             # but in future implementations this will change, probably, to a N to 1 relationship
#             rospy.logerr("Pulled annotations and associated data don't match (%lu != %lu)",
#                      len(matching_anns), len(matching_data))
#             response.message = "Pulled annotations and associated data don't match"
#             response.result = False
#             return response
#     
#         response.annotations = matching_anns
#         response.data        = matching_data
    
        rospy.loginfo("%lu annotations loaded" % i)
        return self.service_success(response)
        
    def get_annotations_data(self, request):
        response = GetAnnotationsDataResponse()
        
        if len(request.annotation_ids) == 0:
            return self.service_error(response, "No annotation ids on request; you must be kidding!")
        
        query = {'id': {'$in': [unique_id.toHexString(id) for id in request.annotation_ids]}}                
        matching_data = self.data_collection.query(query)
        rospy.logdebug("Load annotations data with query %s" % query)

        i = 0
        while True:
            try:
                response.data.append(matching_data.next()[0])
                i += 1
            except StopIteration:
                if (i == 0):
                     # we don't consider this an error
                    rospy.loginfo("No data found for %d requested annotations" % len(request.annotation_ids))
                else:
                    rospy.loginfo("%d objects found for %d annotations" % (i, len(request.annotation_ids)))
                break

        return self.service_success(response)
        
    def pub_annotations_data(self, request):
        response = PubAnnotationsDataResponse()
        
        if len(request.annotation_ids) == 0:
            return self.service_error(response, "No annotation ids on request; you must be kidding!")

        # Verify that all annotations on list belong to the same type (as we will publish them in
        # the same topic) and that at least one is really present in database
        query = {'data_id': {'$in': [unique_id.toHexString(id) for id in request.annotation_ids]}}
        matching_anns = self.anns_collection.query(query, metadata_only=True)
        while True:
            try:
                # Get annotation metadata; we just need the annotation type
                ann_md = matching_anns.next()
                if 'topic_type' not in locals():
                    topic_type = ann_md['type']
                elif topic_type != ann_md['type']:
                    return self.service_error(response, "Cannot publish annotations of different types (%s, %s)"
                                              % (topic_type, ann_md['type']))
            except StopIteration:
                break

        if 'topic_type' not in locals():
            return self.service_error(response, "None of the %d requested annotations was found in database"
                                      % len(request.annotation_ids))

        # Keep the class of the messages to be published; we need it later when deserializing them
        msg_class = roslib.message.get_message_class(topic_type)
        if msg_class is None:
            # This happens if the topic type is wrong or not known for the server (i.e. the package describing it is
            # absent from ROS_PACKAGE_PATH). The second condition is a tricky one, as is a big known limitation of WCF
            # (https://github.com/corot/world_canvas/issues/5)
            return self.service_error(response, "Topic type %s definition not found" % topic_type)
        
        # Advertise a topic with message type request.topic_type if we will publish results as a list (note that we
        # ignore list's type) or use the retrieved annotations type otherwise (we have verified that it's unique) 
        if request.pub_as_list:
            topic_type = request.topic_type
            topic_class = roslib.message.get_message_class(topic_type)
            if topic_class is None:
                # Same comment as in previous service_error call applies here
                return self.service_error(response, "Topic type %s definition not found" % topic_type)
        else:
            topic_class = msg_class

        pub = rospy.Publisher(request.topic_name, topic_class, latch=True, queue_size=5)
        
        # Now retrieve data associated to the requested annotations; reuse query to skip toHexString calls
        query['id'] = query.pop('data_id')                
        matching_data = self.data_collection.query(query)
        rospy.logdebug("Publish data for annotations on query %s" % query)
    
        i = 0
        object_list = list()
        while True:
            try:
                # Get annotation data and deserialize data field to get the original message of type request.topic_type
                ann_data = matching_data.next()[0]
                ann_msg = deserialize_msg(ann_data.data, msg_class)
                if request.pub_as_list:
                    object_list.append(ann_msg)
                else:
                    pub.publish(ann_msg)
                    
                i += 1
            except SerializationError as e:
                rospy.logerr("Deserialization failed: %s" % str(e))
                continue
            except StopIteration:
                if (i == 0):
                    # This must be an error cause we verified before that at least one annotation is present!
                    return self.service_error(response, "No data found for %d requested annotations"
                                              % len(request.annotation_ids))
                if i != len(request.annotation_ids):
                    # Don't need to be an error, as multiple annotations can reference the same data
                    rospy.logwarn("Only %d objects found for %d annotations" % (i, len(request.annotation_ids)))
                else:
                    rospy.loginfo("%d objects found for %d annotations" % (i, len(request.annotation_ids)))
                if request.pub_as_list:
                    pub.publish(object_list)
                break

        return self.service_success(response)

    def delete_annotations(self, request):
        '''
          Deletes the given annotations and its data from database.

          @param request: Service request.
        '''
        response = DeleteAnnotationsResponse()
        
        if len(request.annotations) == 0:
            return self.service_error(response, "No annotation ids on request; you must be kidding!")
        
        annot_data_ids = [a.data_id for a in request.annotations]
        query = {'id': {'$in': [unique_id.toHexString(id) for id in annot_data_ids]}}
        rospy.logdebug("Removing %d annotations data with query %s" % (len(annot_data_ids), query))
        data_removed = self.data_collection.remove(query)
        
        annotation_ids = [a.id for a in request.annotations]
        query = {'id': {'$in': [unique_id.toHexString(id) for id in annotation_ids]}}
        rospy.logdebug("Removing %d annotations with query %s" % (len(annotation_ids), query))
        removed = self.anns_collection.remove(query)
        rospy.loginfo("%d annotations and %d data removed from database" % (removed, data_removed))
        
        if removed != len(annotation_ids):
            # Not all the doomed annotations where found on database. That's not terrible; can happen
            # easily, for example as explained here: https://github.com/corot/world_canvas/issues/38
            # TODO: but we should notify the client lib somehow
            rospy.logwarn("Requested (%d) and deleted (%d) annotations counts doesn't match"
                          % (len(annotation_ids), removed))
        
        return self.service_success(response)

    def save_annotations_data(self, request):
        '''
          Legacy method kept for debug purposes: saves together annotations and its data
          assuming a 1-1 relationship.

          @param request: Service request.
        '''
        response = SaveAnnotationsDataResponse()

        print request.annotations
        for annotation, data in zip(request.annotations, request.data):
            
            # Compose metadata: mandatory fields
            metadata = { 'world'   : annotation.world,
                         'data_id' : unique_id.toHexString(annotation.data_id),
                         'id'      : unique_id.toHexString(annotation.id),
                         'name'    : annotation.name,
                         'type'    : annotation.type
                       }

            # Optional fields; note that both are stored as lists of strings
            if len(annotation.keywords) > 0:
                metadata['keywords'] = annotation.keywords
            if len(annotation.relationships) > 0:
                metadata['relationships'] = [unique_id.toHexString(r) for r in annotation.relationships]

            # Data metadata: just the object id, as all querying is done over the annotations
            data_metadata = { 'id' : unique_id.toHexString(annotation.data_id) }

            rospy.logdebug("Saving annotation %s for world %s" % (annotation.id, annotation.world))

            # Insert both annotation and associated data to the appropriate collection
            self.anns_collection.remove({'id': {'$in': [unique_id.toHexString(annotation.id)]}})
            self.anns_collection.insert(annotation, metadata)
            self.data_collection.remove({'id': {'$in': [unique_id.toHexString(annotation.data_id)]}})
            self.data_collection.insert(data, data_metadata)

        rospy.loginfo("%lu annotations saved" % len(request.annotations))
        return self.service_success(response)

    def list_worlds(self, request):
        response = ListWorldsResponse()
        
        # Query metadata for all annotations in database, shorted by
        # world so we simplify the creation of the list of worlds
        anns_metadata = self.anns_collection.query({}, metadata_only=True, sort_by='world')
        while True:
            try:
                metadata = anns_metadata.next()
                if response.names[-1] != metadata['world']:
                    response.names.append(metadata['world'])
            except IndexError:
                response.names.append(metadata['world'])
            except StopIteration:
                return response

    def set_keyword(self, request):
        response = SetKeywordResponse()
        annot_id, metadata = self.get_metadata(request.id)
        
        if metadata is None:
            response.message = "Annotation not found" 
            response.result = False
            return response

        if request.action == SetKeywordRequest.ADD:
            return self.add_element(annot_id, request.keyword, metadata, 'keywords', response)
        elif request.action == SetKeywordRequest.DEL:
            return self.del_element(annot_id, request.keyword, metadata, 'keywords', response)
        else:
            # Sanity check against crazy service clients
            rospy.logerr("Invalid action %d", request.action)
            response.message = "Invalid action: %d" % request.action
            response.result = False
            return response


    def set_relationship(self, request):
        response = SetRelationshipResponse()
        annot_id, metadata = self.get_metadata(request.id)
        relat_id = unique_id.toHexString(request.relationship)
        
        if metadata is None:
            response.message = 'Annotation not found' 
            response.result = False
            return response

        if request.action == SetRelationshipRequest.ADD:
            return self.add_element(annot_id, relat_id, metadata, 'relationships', response)
        elif request.action == SetRelationshipRequest.DEL:
            return self.del_element(annot_id, relat_id, metadata, 'relationships', response)
        else:
            # Sanity check against crazy service clients
            rospy.logerr("Invalid action: %d" % request.action)
            response.message = "Invalid action: %d" % request.action
            response.result = False
            return response


    def add_element(self, annot_id, element, metadata, md_field, response):

        # Look on metadata for md_field field, for the target element
        field = metadata.get(md_field)

        # Add the new element to metadata field 
        if field is None:
            field = []
        if element not in field:
            field.append(element)
            metadata[md_field] = field
            self.anns_collection.update(metadata)
            rospy.loginfo("%s added to %s for annotation %s" % (element, md_field, annot_id))
        else:
            # Already present; nothing to do (not an error)
            rospy.loginfo("%s already set on %s for annotation %s" % (element, md_field, annot_id))

        return self.service_success(response)

    def del_element(self, annot_id, element, metadata, md_field, response):
        # Look on metadata for md_field field, for the target element
        field = metadata.get(md_field)

        # Remove the element from metadata field, if already present 
        if field is None or element not in field:
            # Requested element not found; nothing to do, but we answer as error
            rospy.logerr("%s not found on %s for annotation %s" % (element, md_field, annot_id))
            response.message = element + ' not found'
            response.result = False
            return response
        else:
            # Found the requested element; remove from metadata
            rospy.loginfo("%s deleted on %s for annotation %s" % (element, md_field, annot_id))
            field.remove(element)
            if len(field) == 0:
                del metadata[md_field]
            else:
                metadata[md_field] = field
            self.anns_collection.update(metadata)

        # No error so return success
        return self.service_success(response)

    def reset_database(self, request):
        # Clear existing database content
        self.world_collection.remove({})
        self.anns_collection.remove({})
        self.data_collection.remove({})

        # No error so return success
        return self.service_success(ResetDatabaseResponse(), 'Database cleared!')


    ##########################################################################
    # Auxiliary methods
    ##########################################################################

    def get_metadata(self, uuid):
        # Get metadata for the given annotation id
        annot_id = unique_id.toHexString(uuid)
        matching_anns = self.anns_collection.query({'id': {'$in': [annot_id]}}, True)

        try:
            return annot_id, matching_anns.next()
        except StopIteration:
            rospy.logwarn("Annotation %s not found" % annot_id)
            return annot_id, None

    def service_success(self, response, message=None):
        if message is not None:
            rospy.loginfo(message)
        response.result = True
        return response

    def service_error(self, response, message):
        rospy.logerr(message)
        response.message = message
        response.result = False
        return response
