/*
 * annotation_collection.cpp
 *
 *  Created on: May 7, 2014
 *      Author: jorge
 */

#include <ros/ros.h>
#include <world_canvas_msgs/GetAnnotations.h>
#include <world_canvas_msgs/GetAnnotationsData.h>
#include <world_canvas_msgs/PubAnnotationsData.h>
#include <world_canvas_msgs/DeleteAnnotations.h>
#include <world_canvas_msgs/SaveAnnotationsData.h>
#include <visualization_msgs/MarkerArray.h>

#include "world_canvas_client_cpp/annotation_collection.hpp"


namespace wcf
{

AnnotationCollection::AnnotationCollection(const std::string& world,
                                           const std::string& srv_namespace)
  : AnnotationCollection(FilterCriteria(world), srv_namespace)
{
}

AnnotationCollection::AnnotationCollection(const FilterCriteria& criteria,
                                           const std::string& srv_namespace)
  : WorldCanvasClient(srv_namespace), filter(criteria)
{
  // Filter parameters provided, so don't wait more to retrieve annotations!
  this->filterBy(criteria);
}

AnnotationCollection::~AnnotationCollection()
{
}


bool AnnotationCollection::filterBy(const FilterCriteria& criteria)
{
  this->filter = criteria;

  try
  {
    ros::ServiceClient client =
        this->getServiceHandle<world_canvas_msgs::GetAnnotations>("get_annotations");

    ROS_INFO("Getting annotations for world %s and additional filter criteria",
             this->filter.getWorld().c_str());
    world_canvas_msgs::GetAnnotations srv;
    srv.request.world         = this->filter.getWorld();
    srv.request.ids           = this->filter.getUuids();
    srv.request.names         = this->filter.getNames();
    srv.request.types         = this->filter.getTypes();
    srv.request.keywords      = this->filter.getKeywords();
    srv.request.relationships = this->filter.getRelationships();
    if (client.call(srv))
    {
      if (srv.response.result == true)
      {
        if (srv.response.annotations.size() > 0)
        {
          ROS_INFO("%lu annotations found", srv.response.annotations.size());
        }
        else
        {
          ROS_INFO("No annotations found for world %s with the given search criteria",
                   this->filter.getWorld().c_str());
        }
        this->annotations = srv.response.annotations;
        return true;
      }
      else
      {
        ROS_ERROR("Server reported an error: %s", srv.response.message.c_str());
        return false;
      }
    }
    else
    {
      ROS_ERROR("Failed to call get_annotations service");
      return false;
    }
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR("ROS exception caught: %s", e.what());
    return false;
  }
}

bool AnnotationCollection::load()
{
  // Retrieve annotations with current filter parameters
  return this->filterBy(this->filter);
}

bool AnnotationCollection::loadData()
{
  if (this->annotations.size() == 0)
  {
    ROS_ERROR("No annotations retrieved. Nothing to load!");
    return false;
  }

  try
  {
    ros::ServiceClient client =
        this->getServiceHandle<world_canvas_msgs::GetAnnotationsData>("get_annotations_data");

    // Request from server the data for annotations previously retrieved; note that we send data
    // uuids, that identify the data associated to the annotation instead of the annotation itself
    ROS_INFO("Loading data for the %lu retrieved annotations", this->annotations.size());
    world_canvas_msgs::GetAnnotationsData srv;
    srv.request.annotation_ids = this->getAnnotsDataIDs();
    if (client.call(srv))
    {
      if (srv.response.result == true)
      {
        if (srv.response.data.size() > 0)
        {
          ROS_INFO("%lu annotations data found", srv.response.data.size());
        }
        else
        {
          ROS_INFO("No data found for the %lu retrieved annotations", this->annotations.size());
        }
        this->annots_data = srv.response.data;
        return true;
      }
      else
      {
        ROS_ERROR("Server reported an error: %s", srv.response.message.c_str());
        return false;
      }
    }
    else
    {
      ROS_ERROR("Failed to call get_annotations_data service");
      return false;
    }
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR("ROS exception caught: %s", e.what());
    return false;
  }
}

bool AnnotationCollection::save()
{
  try
  {
    ros::ServiceClient client =
        this->getServiceHandle<world_canvas_msgs::SaveAnnotationsData>("save_annotations_data");

    // Request server to save current annotations list, with its data
    ROS_INFO("Requesting server to save annotations");
    world_canvas_msgs::SaveAnnotationsData srv;
  //  srv.request.annotations = this->annotations;
  //  srv.request.data        = this->annots_data;

    // This brittle saving procedure requires parallelly ordered annotations and data vectors
    // As this don't need to be the case, we must short them; but we need a better saving procedure (TODO)
    for (unsigned int i = 0; i < this->annotations.size(); i++)
    {
      for (unsigned int j = 0; j < this->annots_data.size(); j++)
      {
        if (this->annots_data[j].id.uuid == this->annotations[i].data_id.uuid)
        {
          ROS_DEBUG("Add annotation for saving with uuid '%s'", unique_id::toHexString(this->annotations[i].id).c_str());
          ROS_DEBUG("Add annot. data for saving with uuid '%s'", unique_id::toHexString(this->annots_data[j].id).c_str());
          srv.request.annotations.push_back(this->annotations[i]);
          srv.request.data.push_back(this->annots_data[j]);
          break;
        }
      }
    }

    // Do at least a rudimentary check
    if (! (this->annotations.size() == this->annots_data.size() == srv.request.annotations.size() == srv.request.data.size()))
    {
      ROS_ERROR("Incoherent annotation and data sizes: %lu != %lu != %lu != %lu",
                this->annotations.size(), this->annots_data.size(), srv.request.annotations.size(), srv.request.data.size());
    }

    bool result = false;
    if (client.call(srv))
    {
      if (srv.response.result == true)
      {
        result = true;
      }
      else
      {
        ROS_ERROR("Server reported an error: %s", srv.response.message.c_str());
      }
    }
    else
    {
      ROS_ERROR("Failed to call save_annotations_data service");
    }

    if (result == true)
      saved = true;

    return result && saveDeletes();
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR("ROS exception caught: %s", e.what());
    return false;
  }
}

bool AnnotationCollection::saveDeletes()
{
  // We remove from database the annotations doomed by delete method, if any
  if (annots_to_delete.size() == 0)
    return true;

  try
  {
    ros::ServiceClient client =
        this->getServiceHandle<world_canvas_msgs::DeleteAnnotations>("delete_annotations");

    // Request server to save current annotations list, with its data
    ROS_INFO("Requesting server to delete annotations");
    world_canvas_msgs::DeleteAnnotations srv;
    srv.request.annotations = annots_to_delete;
    if (client.call(srv))
    {
      if (srv.response.result == true)
      {
        return true;
      }
      else
      {
        ROS_ERROR("Server reported an error: %s", srv.response.message.c_str());
        return false;
      }
    }
    else
    {
      ROS_ERROR("Failed to call delete_annotations service");
      return false;
    }
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR("ROS exception caught: %s", e.what());
    return false;
  }
}

bool AnnotationCollection::add(const world_canvas_msgs::Annotation& annotation,
                               const world_canvas_msgs::AnnotationData& annot_data)
{
  if (annotation.data_id.uuid != annot_data.id.uuid)
  {
    ROS_ERROR("Incoherent annotation and data uuids '%s' != '%s'",
              unique_id::toHexString(annotation.id).c_str(), unique_id::toHexString(annot_data.id).c_str());
    return false;
  }

  for (unsigned int i = 0; i < this->annotations.size(); i++)
  {
    if (this->annotations[i].id.uuid == annotation.id.uuid)
    {
      ROS_ERROR("Duplicated annotation with uuid '%s'", unique_id::toHexString(annotation.id).c_str());
      return false;
    }
  }

  for (unsigned int i = 0; i < this->annots_data.size(); i++)
  {
    if (this->annots_data[i].id.uuid == annot_data.id.uuid)
    {
      ROS_ERROR("Duplicated annotation data with uuid '%s'", unique_id::toHexString(annot_data.id).c_str());
      return false;
    }
  }

  this->annotations.push_back(annotation);
  this->annots_data.push_back(annot_data);

  // Re-publish annotations' visual markers to reflect the incorporation
  this->publishMarkers("annotation_markers");

  saved = false;

  return true;
}

bool AnnotationCollection::update(const world_canvas_msgs::Annotation& annotation,
                                  const world_canvas_msgs::AnnotationData& annot_data)
{
  if (annotation.data_id.uuid != annot_data.id.uuid)
  {
    ROS_ERROR("Incoherent annotation and data uuids '%s' != '%s'",
              unique_id::toHexString(annotation.id).c_str(), unique_id::toHexString(annot_data.id).c_str());
    return false;
  }

  bool found = false;
  for (unsigned int i = 0; i < this->annotations.size(); i++)
  {
    if (this->annotations[i].id.uuid == annotation.id.uuid)
    {
      this->annotations[i] = annotation;
      found = true;
      break;
    }
  }

  if (found == false)
  {
    ROS_ERROR("Annotation uuid '%s' not found", unique_id::toHexString(annotation.id).c_str());
    return false;
  }

  found = false;
  for (unsigned int i = 0; i < this->annots_data.size(); i++)
  {
    if (this->annots_data[i].id.uuid == annot_data.id.uuid)
    {
      this->annots_data[i] = annot_data;
      found = true;
      break;
    }
  }

  if (found == false)
  {
    ROS_ERROR("Annotation data uuid '%s' not found", unique_id::toHexString(annot_data.id).c_str());
    return false;
  }

  // Re-publish annotations' visual markers to reflect changes
  this->publishMarkers("annotation_markers");

  saved = false;

  return true;
}

bool AnnotationCollection::remove(const uuid_msgs::UniqueID& id)
{
  for (unsigned int i = 0; i < this->annotations.size(); i++)
  {
    if (this->annotations[i].id.uuid == id.uuid)
    {
      ROS_DEBUG("Annotation '%s' found", unique_id::toHexString(id).c_str());

      for (unsigned int j = 0; j < this->annots_data.size(); j++)
      {
        if (this->annots_data[j].id.uuid == this->annotations[i].data_id.uuid)
        {
          annots_to_delete.push_back(this->annotations[i]);
          saved = false;

          ROS_DEBUG("Removed annotation with uuid '%s'", unique_id::toHexString(this->annotations[i].id).c_str());
          ROS_DEBUG("Removed annot. data with uuid '%s'", unique_id::toHexString(this->annots_data[j].id).c_str());
          this->annotations.erase(this->annotations.begin() + i);
          this->annots_data.erase(this->annots_data.begin() + j);

          // Re-publish annotations' visual markers to reflect the leave
          this->publishMarkers("annotation_markers");

          return true;
        }
      }

      ROS_ERROR("No data found for annotation '%s' (data uuid is '%s')", unique_id::toHexString(id).c_str(),
                unique_id::toHexString(this->annotations[i].data_id).c_str());
      return false;
    }
  }

  ROS_WARN("Annotation '%s' not found", unique_id::toHexString(id).c_str());
  return false;
}

bool AnnotationCollection::clearMarkers(const std::string& topic)
{
  visualization_msgs::MarkerArray markers_array;
  visualization_msgs::Marker delete_all;
  delete_all.header.frame_id = "/map";  // TODO  OK? seems to work, but I always have /map as ref
  delete_all.action = 3; // visualization_msgs::Marker::DELETEALL is commented but it works!
  markers_array.markers.push_back(delete_all);

  // Check if the given topic has been already advertised for single or multiple markers publishing
  if (endsWith(marker_pub.getTopic(), topic) == true)
  {
    marker_pub.publish(markers_array);
    return true;
  }

  if (endsWith(markers_pub.getTopic(), topic) == true)
  {
    markers_pub.publish(markers_array);
    return true;
  }

  // Topic not yet advertised; do it!
  markers_pub = nh.advertise <visualization_msgs::MarkerArray> (topic, 1, true);
  markers_pub.publish(markers_array);
  return true;
}

bool AnnotationCollection::publishMarkers(const std::string& topic, bool clear_existing)
{
  if (clear_existing == true)
    clearMarkers(topic);

  if (this->annotations.size() == 0)
  {
    ROS_ERROR("No annotations retrieved. Nothing to publish!");
    return false;
  }

  if (endsWith(markers_pub.getTopic(), topic) == false)
  {
    // Advertise a topic for retrieved annotations' visualization markers
    markers_pub = nh.advertise <visualization_msgs::MarkerArray> (topic, 1, true);
  }

  // Process retrieved data to build markers lists
  visualization_msgs::MarkerArray markers_array;
  for (unsigned int i = 0; i < this->annotations.size(); i++)
  {
    markers_array.markers.push_back(makeMarker(i, this->annotations[i]));
    markers_array.markers.push_back(makeLabel(markers_array.markers.back()));
  }

  markers_pub.publish(markers_array);
  return true;
}

bool AnnotationCollection::publishMarker(const std::string& topic, int marker_id,
                                         const world_canvas_msgs::Annotation& ann,
                                         bool clear_existing)
{
  if (endsWith(marker_pub.getTopic(), topic) == false)
  {
    // Advertise a topic to publish a visual marker for the given annotation
    // Use a different publisher from the one created on publishMarkers so both can be used in parallel
    marker_pub = nh.advertise <visualization_msgs::MarkerArray> (topic, 1, true);
  }

  visualization_msgs::MarkerArray markers_array;

  if (clear_existing == true)
    clearMarkers(topic);

  markers_array.markers.push_back(makeMarker(marker_id, ann));
  markers_array.markers.push_back(makeLabel(markers_array.markers.back()));
  marker_pub.publish(markers_array);

  return true;
}

visualization_msgs::Marker AnnotationCollection::makeMarker(int id, const world_canvas_msgs::Annotation& ann)
{
  std::stringstream name; name << ann.name << " [" << ann.type << "]";

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

  return marker;
}

visualization_msgs::Marker AnnotationCollection::makeLabel(const visualization_msgs::Marker& marker)
{
  visualization_msgs::Marker label = marker;
  label.id = marker.id + 1000000;  // marker id must be unique
  label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  label.pose.position.z = marker.pose.position.z + marker.scale.z/2.0 + 0.1; // just above the visual
  label.text = marker.ns != " []" ? marker.ns : "";
  label.scale.x = label.scale.y = label.scale.z = 0.12;
  label.color = marker.color;

  return label;
}


bool AnnotationCollection::publish(const std::string& topic_name, const std::string& topic_type,
                                   bool by_server, bool as_list)
{
  if (this->annotations.size() == 0)
  {
    ROS_ERROR("No annotations retrieved. Nothing to publish!");
    return false;
  }

  std::string common_tt = topic_type;

  if (common_tt.empty() == true)
  {
    if (as_list == true)
    {
      ROS_ERROR("Topic type argument is mandatory if as_list is true");
      return false;
    }
    else
    {
      // Take annotations type and verify that it's the same within the
      // collection (as we will publish all of them in the same topic)
      for (unsigned int i = 0; i < this->annotations.size(); i++)
      {
        if ((common_tt.empty() == false) && (common_tt != annotations[i].type))
        {
          ROS_ERROR("Cannot publish annotations of different types (%s, %s)",
                    common_tt.c_str(), annotations[i].type.c_str());
          return false;
        }
        common_tt = annotations[i].type;
      }
    }
  }

  try
  {
    if (by_server == true)
    {
      ros::ServiceClient client =
          this->getServiceHandle<world_canvas_msgs::PubAnnotationsData>("pub_annotations_data");

      // Request server to publish the annotations previously retrieved; note that we send the data
      // uuids, that identify the data associated to the annotation instead of the annotation itself
      ROS_INFO("Requesting server to publish annotations of type '%s'", common_tt.c_str());
      world_canvas_msgs::PubAnnotationsData srv;
      if (as_list == true)
      {
        // We try to publish all annotations if the user request to do so as a list, but...
	srv.request.annotation_ids = this->getAnnotsDataIDs();
      }
      else
      {
        // we take only annotations of the given type if we will publish them one by one, as we will
        // publish all of them in the same topic
        // TODO: this is quite confusing: https://github.com/corot/world_canvas_libs/issues/24
        for (unsigned int i = 0; i < this->annotations.size(); i++)
        {
          if (annotations[i].type == common_tt)
            srv.request.annotation_ids.push_back(annotations[i].data_id);
        }
      }
      srv.request.topic_name = topic_name;
      srv.request.topic_type = common_tt;
      srv.request.pub_as_list = as_list;
      if (client.call(srv))
      {
        if (srv.response.result == true)
        {
          return true;
        }
        else
        {
          ROS_ERROR("Server reported an error: %s", srv.response.message.c_str());
          return false;
        }
      }
      else
      {
        ROS_ERROR("Failed to call pub_annotations_data service");
        return false;
      }
    }
    else
    {
      // TODO: we cannot publish here without the messages class, as we did with Python. Maybe I can
      // use templates, as the user of this class knows the message class. Or I can even make this a
      // template class, assuming annotations collections have a uniform type.
      // See https://github.com/corot/world_canvas/issues/5 for details
      ROS_ERROR("Publish by client not implemented!");
      return false;
    }
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR("ROS exception caught: %s", e.what());
    return false;
  }
}

bool AnnotationCollection::hasAnnotation(const UniqueIDmsg& id)
{
  for (unsigned int i = 0; i < annotations.size(); i++)
  {
    if (annotations[i].id.uuid == id.uuid)
      return true;
  }
  return false;
}

const world_canvas_msgs::Annotation& AnnotationCollection::getAnnotation(const UniqueIDmsg& id)
{
  for (unsigned int i = 0; i < annotations.size(); i++)
  {
    if (annotations[i].id.uuid == id.uuid)
      return annotations[i];
  }
  throw ros::Exception("Uuid not found: " + unique_id::toHexString(id));
}

std::vector<world_canvas_msgs::Annotation>
AnnotationCollection::getAnnotations(const std::string& name)
{
  std::vector<world_canvas_msgs::Annotation> result;
  for (unsigned int i = 0; i < annotations.size(); i++)
  {
    if (annotations[i].name == name)
      result.push_back(annotations[i]);
  }
  return result;
}

std::vector<UniqueIDmsg> AnnotationCollection::getAnnotationIDs()
{
  std::vector<UniqueIDmsg> uuids(annotations.size());
  for (unsigned int i = 0; i < annotations.size(); i++)
  {
    uuids[i] = annotations[i].id;
  }
  return uuids;
}

std::vector<UniqueIDmsg> AnnotationCollection::getAnnotsDataIDs()
{
  std::vector<UniqueIDmsg> uuids(annotations.size());
  for (unsigned int i = 0; i < annotations.size(); i++)
  {
    uuids[i] = annotations[i].data_id;
  }
  return uuids;
}

const world_canvas_msgs::AnnotationData&
AnnotationCollection::getData(const world_canvas_msgs::Annotation& ann)
{
  for (unsigned int i = 0; i < this->annots_data.size(); i++)
  {
    if (this->annots_data[i].id.uuid == ann.data_id.uuid)
    {
      return this->annots_data[i];
    }
  }

  throw ros::Exception("Data uuid not found: " + unique_id::toHexString(ann.data_id));
}

} // namespace wcf
