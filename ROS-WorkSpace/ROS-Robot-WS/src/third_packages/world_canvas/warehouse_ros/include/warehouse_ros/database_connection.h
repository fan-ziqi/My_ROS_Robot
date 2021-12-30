/*
 * Copyright (c) 2015, Fetch Robotics
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
 * The DatabaseConnection class
 *
 * \author Connor Brew
 */

#ifndef WAREHOUSE_ROS_DATABASE_CONNECTION_H
#define WAREHOUSE_ROS_DATABASE_CONNECTION_H

#include <warehouse_ros/message_collection.h>

namespace warehouse_ros
{
class DatabaseConnection
{
public:
  virtual ~DatabaseConnection()
  {
  }

  /// \brief Set database connection params.
  virtual bool setParams(const std::string& host, unsigned port, float timeout = 60.0) = 0;

  /// \brief Set database connection params.
  virtual bool setTimeout(float timeout) = 0;

  /// Setup the database connection. This call assumes setParams() has been previously called.
  /// Returns true if the connection was succesfully established.
  virtual bool connect() = 0;

  /// Returns whether the database is connected.
  virtual bool isConnected() = 0;

  /// \brief Drop a db and all its collections.
  /// A DbClientConnection exception will be thrown if the database is not connected.
  virtual void dropDatabase(const std::string& db_name) = 0;

  /// \brief Return the ROS Message type of a given collection
  virtual std::string messageType(const std::string& db_name, const std::string& collection_name) = 0;

  /// \brief Open a collection on the DB.  The collection is created if it doesn't exist.
  /// A DbClientConnection exception will be thrown if the database is not connected.
  template <class M>
  MessageCollection<M> openCollection(const std::string& db_name, const std::string& collection_name);

  /// \brief Open a collection on the DB.  The collection is created if it doesn't exist.
  /// A DbClientConnection exception will be thrown if the database is not connected.
  template <class M>
  typename MessageCollection<M>::Ptr openCollectionPtr(const std::string& db_name, const std::string& collection_name);

  typedef boost::shared_ptr<DatabaseConnection> Ptr;

protected:
  virtual MessageCollectionHelper::Ptr openCollectionHelper(const std::string& db_name,
                                                            const std::string& collection_name) = 0;
};

}  // namespace

#include "impl/database_connection_impl.hpp"

#endif  // include guard
