/*
 * world_canvas_client.hpp
 *
 *  Created on: Oct 13, 2014
 *      Author: jorge
 */

#ifndef WORLD_CANVAS_CLIENT_HPP_
#define WORLD_CANVAS_CLIENT_HPP_

#include <ros/ros.h>

namespace wcf
{

/**
 * Base class for clients of the world canvas server.
 */
class WorldCanvasClient
{
protected:
  ros::NodeHandle nh;
  std::string srv_namespace;


  /**
   * Initializes the required stuff to interface with the world canvas server.
   *
   * @param srv_namespace: World canvas handles can be found under this namespace.
   */
  WorldCanvasClient(const std::string& srv_namespace = "")
  {
    this->srv_namespace = srv_namespace;
    if (this->srv_namespace.size() == 0 || this->srv_namespace.back() != '/')
      this->srv_namespace.push_back('/');
  }

  /**
   * Create a service client of the template type and wait until the service is available.
   *
   * @param service_name: ROS service name to get, without namespace.
   * @param timeout: Timeout to wait for the service to come up.
   * @returns: The service handle.
   * @throws: ROS exception on timeout.
   */
  template <typename T>
  ros::ServiceClient getServiceHandle(const std::string& service_name, double timeout = 5.0)
  {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<T>(srv_namespace + service_name);
    ROS_INFO("Waiting for '%s' service...", service_name.c_str());
    if (client.waitForExistence(ros::Duration(timeout)) == false)
    {
      ROS_ERROR("'%s' service not available after %.2f s", service_name.c_str(), timeout);
      throw ros::Exception(service_name + " service not available");
    }

    return client;
  }

};

} // namespace wcf

#endif /* WORLD_CANVAS_CLIENT_HPP_ */
