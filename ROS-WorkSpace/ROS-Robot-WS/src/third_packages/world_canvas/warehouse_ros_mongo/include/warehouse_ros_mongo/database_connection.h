/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *
 */

/**
 * \file
 *
 * Db-level operations.  Most operations are in message_collection.h
 *
 * \author Bhaskara Marthi
 */

#ifndef WAREHOUSE_ROS_MONGO_DATABASE_CONNECTION_H
#define WAREHOUSE_ROS_MONGO_DATABASE_CONNECTION_H

#include <warehouse_ros/database_connection.h>
#include <warehouse_ros_mongo/message_collection.h>
#include <boost/shared_ptr.hpp>

namespace warehouse_ros_mongo
{
class MongoDatabaseConnection : public warehouse_ros::DatabaseConnection
{
public:
  MongoDatabaseConnection();

  bool setParams(const std::string& host, unsigned port, float timeout);

  bool setTimeout(float timeout);

  bool connect();

  bool isConnected();

  void dropDatabase(const std::string& db_name);

  std::string messageType(const std::string& db_name, const std::string& collection_name);

protected:
  boost::shared_ptr<mongo::DBClientConnection> conn_;

  std::string host_;
  unsigned port_;
  float timeout_;

  MessageCollectionHelper::Ptr openCollectionHelper(const std::string& db_name, const std::string& collection_name);
};

}  // namespace

#endif  // include guard
