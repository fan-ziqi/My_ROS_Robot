/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Fetch Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Connor Brew */

#include <warehouse_ros/database_loader.h>

namespace warehouse_ros
{
using std::string;

DatabaseLoader::DatabaseLoader() : nh_("~")
{
  initialize();
}

DatabaseLoader::~DatabaseLoader()
{
}

void DatabaseLoader::initialize()
{
  // Create the plugin loader.
  try
  {
    db_plugin_loader_.reset(new pluginlib::ClassLoader<DatabaseConnection>("warehouse_ros", "warehouse_ros::"
                                                                                            "DatabaseConnection"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating database_connection plugin loader " << ex.what());
  }
}

typename DatabaseConnection::Ptr DatabaseLoader::loadDatabase()
{
  if (!db_plugin_loader_)
  {
    return typename DatabaseConnection::Ptr(new DBConnectionStub());
  }

  // Search for the warehouse_plugin parameter in the local namespace of the node, and up the tree of namespaces.
  // If the desired param is not found, make a final attempt to look for the param in the default namespace
  string paramName;
  if (!nh_.searchParam("warehouse_plugin", paramName))
    paramName = "warehouse_plugin";
  string db_plugin;
  if (!nh_.getParamCached(paramName, db_plugin))
  {
    ROS_ERROR("Could not find parameter for database plugin name");
    return typename DatabaseConnection::Ptr(new DBConnectionStub());
  }

  DatabaseConnection::Ptr db;
  try
  {
    db = db_plugin_loader_->createUniqueInstance(db_plugin);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM("Exception while loading database plugin '" << db_plugin << "': " << ex.what() << std::endl);
    return typename DatabaseConnection::Ptr(new DBConnectionStub());
  }

  bool hostFound = false;
  bool portFound = false;

  if (!nh_.searchParam("warehouse_host", paramName))
    paramName = "warehouse_host";
  std::string host;
  if (nh_.getParamCached(paramName, host))
  {
    hostFound = true;
  }

  if (!nh_.searchParam("warehouse_port", paramName))
    paramName = "warehouse_port";
  int port;
  if (nh_.getParamCached(paramName, port))
  {
    portFound = true;
  }

  if (hostFound && portFound)
  {
    db->setParams(host, port);
  }

  return db;
}

MessageCollectionHelper::Ptr DBConnectionStub::openCollectionHelper(const std::string& db_name,
                                                                    const std::string& collection_name)
{
  return MessageCollectionHelper::Ptr();
}
}
