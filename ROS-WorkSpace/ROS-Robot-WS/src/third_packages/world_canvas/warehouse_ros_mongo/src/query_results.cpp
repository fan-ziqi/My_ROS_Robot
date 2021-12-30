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
 * Implementation of MongoResultIterator.
 *
 * \author Bhaskara Marthi
 */

#include <warehouse_ros_mongo/query_results.h>

namespace warehouse_ros_mongo
{
MongoResultIterator::MongoResultIterator(boost::shared_ptr<mongo::DBClientConnection> conn,
                                         boost::shared_ptr<mongo::GridFS> gfs, const std::string& ns,
                                         const mongo::Query& query)
  : cursor_(new Cursor(conn->query(ns, query))), gfs_(gfs)
{
  if ((*cursor_)->more())
    next_ = (*cursor_)->nextSafe();
}

bool MongoResultIterator::next()
{
  ROS_ASSERT(next_);
  if ((*cursor_)->more())
  {
    next_ = (*cursor_)->nextSafe();
    return true;
  }
  else
  {
    next_.reset();
    return false;
  }
}

bool MongoResultIterator::hasData() const
{
  return (bool)next_;
}

warehouse_ros::Metadata::ConstPtr MongoResultIterator::metadata() const
{
  ROS_ASSERT(next_);
  return typename warehouse_ros::Metadata::ConstPtr(new MongoMetadata(next_->copy()));
}

std::string MongoResultIterator::message() const
{
  mongo::OID blob_id;
  (*next_)["blob_id"].Val(blob_id);
  mongo::BSONObj q = BSON("_id" << blob_id);
  mongo::GridFile f = gfs_->findFile(q);
  ROS_ASSERT(f.exists());
  std::stringstream ss(std::ios_base::out);
  f.write(ss);
  return ss.str();
}

mongo::BSONObj MongoResultIterator::metadataRaw() const
{
  ROS_ASSERT(next_);
  return next_->copy();
}

}  // namespace
