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
 * Implementation of warehouse_ros::ResultIteratorHelper for mongo queries
 *
 * \author Bhaskara Marthi
 */

#ifndef WAREHOUSE_ROS_MONGO_QUERY_RESULTS_H
#define WAREHOUSE_ROS_MONGO_QUERY_RESULTS_H

#include <warehouse_ros_mongo/metadata.h>
#include <mongo/client/dbclientcursor.h>
#include <warehouse_ros/query_results.h>
#include <boost/optional.hpp>
#include <memory>

namespace warehouse_ros_mongo
{
// To avoid some const-correctness issues we wrap Mongo's returned auto_ptr in
// another pointer
typedef std::auto_ptr<mongo::DBClientCursor> Cursor;
typedef boost::shared_ptr<Cursor> CursorPtr;

class MongoResultIterator : public warehouse_ros::ResultIteratorHelper
{
public:
  MongoResultIterator(boost::shared_ptr<mongo::DBClientConnection> conn, boost::shared_ptr<mongo::GridFS> gfs,
                      const std::string& ns, const mongo::Query& query);
  bool next();
  bool hasData() const;
  warehouse_ros::Metadata::ConstPtr metadata() const;
  std::string message() const;
  mongo::BSONObj metadataRaw() const;

private:
  CursorPtr cursor_;
  boost::optional<mongo::BSONObj> next_;
  boost::shared_ptr<mongo::GridFS> gfs_;
};

}  // namespace

#endif  // include guard
