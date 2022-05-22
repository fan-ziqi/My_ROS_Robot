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
 * The MessageCollection class
 *
 * \author Bhaskara Marthi
 */

#ifndef WAREHOUSE_ROS_MONGO_MESSAGE_COLLECTION_H
#define WAREHOUSE_ROS_MONGO_MESSAGE_COLLECTION_H

#include <warehouse_ros/message_collection.h>
#include <warehouse_ros_mongo/query_results.h>

namespace warehouse_ros_mongo
{
using warehouse_ros::Metadata;
using warehouse_ros::Query;
using warehouse_ros::MessageCollectionHelper;
using warehouse_ros::ResultIteratorHelper;

class MongoMessageCollection : public warehouse_ros::MessageCollectionHelper
{
public:
  MongoMessageCollection(boost::shared_ptr<mongo::DBClientConnection> conn, const std::string& db_name,
                         const std::string& collection_name);

  bool initialize(const std::string& datatype, const std::string& md5);

  /// \post Ensure that there's an index on the given field.
  /// Note that index on _id and creation_time are always created.
  void ensureIndex(const std::string& field);

  /// \brief Insert a ROS message, together with some optional metadata,
  /// into the db
  /// \throws mongo::DBException if unable to insert
  void insert(char* msg, size_t msg_size, Metadata::ConstPtr metadata);

  /// \retval Iterator range over matching messages
  /// \param query A metadata object representing a query.
  /// \param metadata_only If this is true, only retrieve the metadata
  /// (returned message objects will just be default constructed)
  ResultIteratorHelper::Ptr query(Query::ConstPtr query, const std::string& sort_by, bool ascending) const;

  /// \brief Remove messages matching query
  unsigned removeMessages(Query::ConstPtr query);

  /// \brief Modify metadata
  /// Find message matching \a q and update its metadata using \a m
  /// In other words, overwrite keys in the message using \a m, but
  /// keep keys that don't occur in \a m.
  void modifyMetadata(Query::ConstPtr q, Metadata::ConstPtr m);

  /// \brief Count messages in collection
  unsigned count();

  /// \brief Return name of collection
  std::string collectionName() const;

  Query::Ptr createQuery() const
  {
    return Query::Ptr(new MongoQuery());
  }

  Metadata::Ptr createMetadata() const
  {
    return Metadata::Ptr(new MongoMetadata());
  }

private:
  void listMetadata(mongo::Query& mquery, std::vector<mongo::BSONObj>& metas);

  inline MongoMetadata& downcastMetadata(Metadata::ConstPtr metadata) const
  {
    return *(const_cast<MongoMetadata*>(static_cast<const MongoMetadata*>(metadata.get())));
  }

  inline MongoQuery& downcastQuery(Query::ConstPtr query) const
  {
    return *(const_cast<MongoQuery*>(static_cast<const MongoQuery*>(query.get())));
  }

  boost::shared_ptr<mongo::DBClientConnection> conn_;
  boost::shared_ptr<mongo::GridFS> gfs_;
  const std::string ns_;
  const std::string db_;
  const std::string coll_;
};

}  // namespace

#endif  // include guard
