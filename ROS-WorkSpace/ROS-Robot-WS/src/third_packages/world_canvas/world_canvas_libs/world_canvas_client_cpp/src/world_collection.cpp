/*
 * world_collection.cpp
 *
 *  Created on: Oct 13, 2014
 *      Author: jorge
 */

#include <ros/ros.h>
#include <world_canvas_msgs/ListWorlds.h>

#include "world_canvas_client_cpp/world_collection.hpp"


namespace wcf
{

WorldCollection::WorldCollection(const std::string& srv_namespace)
  : WorldCanvasClient(srv_namespace)
{
  ros::ServiceClient client =
      this->getServiceHandle<world_canvas_msgs::ListWorlds>("list_worlds");

  world_canvas_msgs::ListWorlds srv;
  if (client.call(srv))
  {
    world_names = srv.response.names;
    ROS_DEBUG("Loaded %lu world names", world_names.size());
  }
  else
  {
    throw ros::Exception("Failed to call list_worlds service");
  }
}


} // namespace wcf
