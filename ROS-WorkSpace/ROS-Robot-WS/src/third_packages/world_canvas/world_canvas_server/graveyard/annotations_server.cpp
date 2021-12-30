/*
 * Copyright (c) 2013, Yujin Robot.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Based on map_store, but adapted to publish/store semantic information (annotations)
behavior:
 - sets up connection to warehouse
 - tells warehouse to publish latest annotations of any session (this is commented by now; can be misleading)
 - spins, handling service calls

service calls:
 - publish_map(map uuid) returns true if annotations were found for the given map
   - queries warehouse for annotations associated to the given map
   - publishes the annotations on markers, tables, columns, walls topics
   - publishes visualization markers for all the annotations
   - sets map uuid as the current map, so we can update published annotations if needed
 - rename_map(map uuid, new name) returns void
   - renames the associated map identified by map_uuid on annotations database
 - delete_map(map uuid) returns true if annotations were found for the given map
   - deletes the annotations associated to the given map
   - if current map is set, calls publish_annotations to reflect changes
 - save_map(map uuid, map name, session id) returns error message if any
   - saves currently published annotations as associated to the given map
   - if current map is set, calls publish_annotations to reflect changes

 NOT IMPLEMENTED, and not useful by now
 - list_maps() returns list of map metadata: {id, name, timestamp, maybe thumbnail}
   - query for all annotations.
 - dynamic_map() returns nav_msgs/OccupancyGrid
   - returns the dynamic map
 */

#include <mongo_ros/message_collection.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <world_canvas_msgs/Annotation.h>
#include <world_canvas_msgs/AnnotationData.h>

#include <world_canvas_msgs/LoadAnnotationsData.h>
#include <world_canvas_msgs/SaveAnnotationsData.h>

#include <string>
#include <sstream>
#include <exception>
#include <uuid/uuid.h>

namespace mr=mongo_ros;

mr::MessageCollection<world_canvas_msgs::Annotation>     *anns_collection;
mr::MessageCollection<world_canvas_msgs::AnnotationData> *data_collection;

ros::Publisher map_pub;
ros::Publisher markers_pub;

visualization_msgs::MarkerArray markers_array;

//world_canvas_msgs::SemanticMap map_msg;

typedef std::vector<mr::MessageWithMetadata<world_canvas_msgs::Annotation>::ConstPtr>     AnnsVector;
typedef std::vector<mr::MessageWithMetadata<world_canvas_msgs::AnnotationData>::ConstPtr> DataVector;

std::string uuid2str(unsigned char* pUuid) {
  uuid_t uuid;
  memcpy(uuid, pUuid, sizeof(uuid_t));
  char uuid_string[37]; // UUID prints into 36 bytes + NULL terminator
  uuid_unparse_lower(uuid, uuid_string);
  return std::string(uuid_string);
}

//
//void onMapReceived(const world_canvas_msgs::SemanticMap::ConstPtr& msg)
//{
//  map_msg = *msg;
//}

void clearMarkers()
{
  for (int i = 0; i < markers_array.markers.size(); ++i)
  {
    markers_array.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  if (markers_array.markers.size() > 0)
  {
    markers_pub.publish(markers_array);
    markers_array.markers.clear();
  }
}

visualization_msgs::Marker makeMarker(int id, const world_canvas_msgs::Annotation& ann)
{
  std::stringstream name; name << ann.type << '/' << ann.name;

  visualization_msgs::Marker marker;
  marker.header.frame_id = ann.pose.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.scale = ann.size;
  marker.color = ann.color;
  marker.ns = name.str();
  marker.id = id;
  marker.pose = ann.pose.pose.pose;
  marker.type = ann.shape;
  marker.action = visualization_msgs::Marker::ADD;

  // Make z-coordinate the lowest point of markers, so they appear to lay in the floor if they
  // have zero z. This is wrong for AR markers, but doesn't matter as they are rather small.
  marker.pose.position.z += marker.scale.z/2.0;

  return marker;
}

visualization_msgs::Marker makeLabel(const visualization_msgs::Marker& marker)
{
  visualization_msgs::Marker label = marker;
  label.id = marker.id + 1000000;  // marker id must be unique
  label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  label.pose.position.z = marker.pose.position.z + marker.scale.z/2.0 + 0.1; // just above the visual
  label.text = marker.ns;
  label.scale.x = label.scale.y = label.scale.z = 0.12;
  // label.color.r = label.color.g = label.color.b = 0.0; label.color.a = 1.0; // make solid black

  return label;
}

//bool publishMap(world_canvas_msgs::PublishMap::Request &request,
//                world_canvas_msgs::PublishMap::Response &response)
//{
//  ROS_INFO("Publish semantic map '%s'", request.map_uuid.c_str());
//
////  last_map = request.map_id;
////  ros::NodeHandle nh;
////  nh.setParam("last_map_id", last_map);
//
//  try
//  {
//    // remove from visualization tools and delete visualization markers
//    clearMarkers();
//
//    MapsVector matching_maps =
//        maps_collection->pullAllResults(mr::Query("map_uuid", request.map_uuid));
//
//    if (matching_maps.size() == 0)
//    {
//      ROS_WARN("No semantic map found for id '%s'; we don't consider this an error",
//               request.map_uuid.c_str());
//      response.found = false;
//      return true;  // we don't consider this an error
//    }
//    else if (matching_maps.size() > 1)
//    {
//      // Extra sanity checking
//      ROS_WARN("More than one (%lu) semantic maps found for id '%s'; we consider only the first one",
//               matching_maps.size(), request.map_uuid.c_str());
//    }
//
//    // At least one map found; publish it
//    response.found = true;
//
//    ROS_INFO("Semantic map fetched containing %lu annotations",
//             matching_maps[0]->annotations.size());
//    map_pub.publish(world_canvas_msgs::SemanticMapConstPtr(matching_maps[0]));
//
//    // compose and publish visualization markers
//    for (int i = 0; i < matching_maps[0]->annotations.size(); ++i)
//    {
//      world_canvas_msgs::Annotation ann = matching_maps[0]->annotations[i];
//      markers_array.markers.push_back(makeMarker(i, ann));
//      markers_array.markers.push_back(makeLabel(markers_array.markers.back()));
//    }
//
//    if (markers_array.markers.size() > 0)
//      markers_pub.publish(markers_array);
//
//    // Keep track of currently published annotations to republish if we receive updated data
//    pub_map_id = request.map_uuid;
//    return true;
//  }
//  catch(const std::exception &e) {
//    ROS_ERROR("Error during query: %s", e.what());
//    return false;
//  }
//
//  return true;
//}

bool loadAnnotationsData(world_canvas_msgs::LoadAnnotationsData::Request &request,
                         world_canvas_msgs::LoadAnnotationsData::Response &response)
{
  try
  {
    AnnsVector matching_anns =
        anns_collection->pullAllResults(mr::Query("map_uuid", request.map_uuid));

    if (matching_anns.size() == 0)
    {
      ROS_INFO("No annotations found for map '%s'; we don't consider this an error",
                request.map_uuid.c_str());
      response.result = true;
      return true;  // we don't consider this an error
    }

    DataVector matching_data =
        data_collection->pullAllResults(mr::Query("map_uuid", request.map_uuid));

    if (matching_anns.size() != matching_data.size())
    {
      // we consider this an error by now, as we assume a 1 to 1 relationship;
      // but in future implementations this will change, probably, to a N to 1 relationship
      ROS_ERROR("Pulled annotations and associated data don't match (%lu != %lu)",
               matching_anns.size(), matching_data.size());
      response.message = "Pulled annotations and associated data don't match";
      response.result = false;
      return false;
    }

    response.annotations.reserve(matching_anns.size());
    response.data.reserve(matching_data.size());
    for (int i = 0; i < matching_anns.size(); ++i)
    {
      response.annotations.push_back(*matching_anns[i]);
      response.data.push_back(*matching_data[i]);
    }

    ROS_INFO("%lu annotations loaded", matching_anns.size());
    response.result = true;
    return true;
  }
  catch(const std::exception &e) {
    ROS_ERROR("Error during query: %s", e.what());
    response.message = e.what();
    response.result = false;
    return false;
  }
}

bool saveAnnotationsData(world_canvas_msgs::SaveAnnotationsData::Request &request,
                         world_canvas_msgs::SaveAnnotationsData::Response &response)
{
  for (int i = 0; i < request.annotations.size(); ++i)
  {
    world_canvas_msgs::Annotation annotation = request.annotations[i];
    world_canvas_msgs::AnnotationData data = request.data[i];
    ///uuid_t kk = static_cast<uuid_t>(annotation.id.uuid.c_array());
    std::string annotation_id = uuid2str(annotation.id.uuid.c_array());
    mr::Metadata metadata = mr::Metadata("map_uuid", annotation.map_uuid,
                                    ///     "world_id", annotation.world_id,
                                         "id",       annotation_id);
//    mr::Metadata metadata = mr::Metadata("timestamp",   request.map_name,
//                                         "map_uuid",   request.map_uuid,
//                                         "world_id", request.session_id,
//                                         "id",   request.map_uuid,
//                                         "name", request.session_id,
//                                         "type", request.session_id);

    ROS_DEBUG("Saving annotation %s for map %s", annotation_id.c_str(), annotation.map_uuid.c_str());
    try
    {
      ////mr::Query q = mr::Query().append("map_uuid", annotation.map_uuid).append("id", annotation.id);
      anns_collection->removeMessages(mr::Query("id", annotation_id));
      anns_collection->insert(annotation, metadata);
      data_collection->removeMessages(mr::Query("id", annotation_id));
      data_collection->insert(data, metadata);
    }
    catch (mongo::DBException& e)
    {
      ROS_ERROR("Error during saving: %s", e.what());
      response.message = e.what();
      response.result = false;
      return false;
    }
  }

  ROS_INFO("%lu annotations saved", request.annotations.size());
  response.result = true;
  return true;
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "annotations_server");
  ros::NodeHandle nh;

  anns_collection = new mr::MessageCollection<world_canvas_msgs::Annotation> ("world_canvas", "annotations");
  anns_collection->ensureIndex("id");

  data_collection = new mr::MessageCollection<world_canvas_msgs::AnnotationData> ("world_canvas", "annotations_data");
  data_collection->ensureIndex("id");

//  map_pub = nh.advertise<world_canvas_msgs::SemanticMap> ("semantics_out", 1, true);
  markers_pub = nh.advertise<visualization_msgs::MarkerArray>  ("visual_markers", 1, true);
//  if (last_map != "")
//  {
//    nav_msgs::OccupancyGridConstPtr map;
//    if (lookupMap(last_map, map))
//    {
//      try {
//	map_publisher.publish(map);
//      } catch(...) {
//	ROS_ERROR("Error publishing map");
//      }
//    }
//    else
//    {
//      ROS_ERROR("Invalid last_map_id");
//    }
//  }

//  ros::Subscriber map_sub = nh.subscribe("semantics_in", 1, onMapReceived);

//  ros::ServiceServer publish_map_srv = nh.advertiseService("publish_map", publishMap);
//  ros::ServiceServer delete_map_srv  = nh.advertiseService("delete_map",  deleteMap);
//  ros::ServiceServer rename_map_srv  = nh.advertiseService("rename_map",  renameMap);
  ros::ServiceServer load_data_srv = nh.advertiseService("load_annotations_data", loadAnnotationsData);
  ros::ServiceServer save_data_srv = nh.advertiseService("save_annotations_data", saveAnnotationsData);

//  NOT IMPLEMENTED, and not useful by now
//  ros::ServiceServer list_map_srv    = nh.advertiseService("list_maps",    listMap);
//  ros::ServiceServer dynamic_map_srv = nh.advertiseService("dynamic_map", dynamicMap);

  ROS_DEBUG("Annotations server running");

  ros::spin();

  delete anns_collection;
  delete data_collection;

  return 0;
}
