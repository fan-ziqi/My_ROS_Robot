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
 * Test script for Mongo ros c++ interface
 *
 * \author Bhaskara Marthi
 */

// %Tag(CPP_CLIENT)%

#include <gtest/gtest.h>
#include <warehouse_ros_mongo/database_connection.h>
#include "test_mongo_helpers.h"

namespace gm = geometry_msgs;
using warehouse_ros::Metadata;
using warehouse_ros::Query;
using warehouse_ros::NoMatchingMessageException;
using std::vector;
using std::string;
using std::cout;

typedef warehouse_ros::MessageCollection<gm::Pose> PoseCollection;
typedef warehouse_ros::MessageWithMetadata<gm::Pose> PoseWithMetadata;
typedef PoseWithMetadata::ConstPtr PoseMetaPtr;

// Helper function that creates metadata for a message.
// Here we'll use the x and y position, as well as a 'name'
// field that isn't part of the original message.
Metadata::Ptr makeMetadata(PoseCollection coll, const gm::Pose& p, const string& n)
{
  Metadata::Ptr meta = coll.createMetadata();
  meta->append("x", p.position.x);
  meta->append("y", p.position.y);
  meta->append("name", n);
  return meta;
}

TEST(MongoRos, MongoRos)
{
  // Set up db
  warehouse_ros_mongo::MongoDatabaseConnection conn;
  conn.setParams("localhost", 27017, 60.0);
  conn.connect();
  ASSERT_TRUE(conn.isConnected());

  // Clear existing data if any
  conn.dropDatabase("my_db");

  // Open the collection
  PoseCollection coll = conn.openCollection<gm::Pose>("my_db", "poses");

  // Arrange to index on metadata fields 'x' and 'name'
  // coll.ensureIndex("name");
  // coll.ensureIndex("x");

  // Add some poses and metadata
  const gm::Pose p1 = makePose(24, 42, 0);
  const gm::Pose p2 = makePose(10, 532, 3);
  const gm::Pose p3 = makePose(53, 22, 5);
  const gm::Pose p4 = makePose(22, -5, 33);
  coll.insert(p1, makeMetadata(coll, p1, "bar"));
  coll.insert(p2, makeMetadata(coll, p2, "baz"));
  coll.insert(p3, makeMetadata(coll, p3, "qux"));
  coll.insert(p1, makeMetadata(coll, p1, "oof"));
  coll.insert(p4, makeMetadata(coll, p4, "ooof"));
  EXPECT_EQ(5u, coll.count());

  // Simple query: find the pose with name 'qux' and return just its metadata
  // Since we're doing an equality check, we don't explicitly specify a predicate
  Query::Ptr q1 = coll.createQuery();
  q1->append("name", "qux");
  vector<PoseMetaPtr> res = coll.queryList(q1, true);
  EXPECT_EQ(1u, res.size());
  EXPECT_EQ("qux", res[0]->lookupString("name"));
  EXPECT_DOUBLE_EQ(53, res[0]->lookupDouble("x"));

  // Set up query: position.x < 40 and position.y > 0.  Reverse order
  // by the "name" metadata field.  Also, here we pull the message itself, not
  // just the metadata.  Finally, we can't use the simplified construction
  // syntax here because it's too long
  Query::Ptr q2 = coll.createQuery();
  q2->appendLT("x", 40);
  q2->appendGT("y", 0);
  vector<PoseMetaPtr> poses = coll.queryList(q2, false, "name", false);

  // Verify poses.
  EXPECT_EQ(3u, poses.size());
  EXPECT_EQ(p1, *poses[0]);
  EXPECT_EQ(p2, *poses[1]);
  EXPECT_EQ(p1, *poses[2]);

  EXPECT_EQ("oof", poses[0]->lookupString("name"));
  EXPECT_EQ("baz", poses[1]->lookupString("name"));
  EXPECT_EQ("bar", poses[2]->lookupString("name"));

  // Set up query to delete some poses.
  Query::Ptr q3 = coll.createQuery();
  q3->appendLT("y", 30);

  EXPECT_EQ(5u, coll.count());
  EXPECT_EQ(2u, coll.removeMessages(q3));
  EXPECT_EQ(3u, coll.count());

  // Test findOne
  Query::Ptr q4 = coll.createQuery();
  q4->append("name", "bar");
  EXPECT_EQ(p1, *coll.findOne(q4, false));
  EXPECT_DOUBLE_EQ(24, coll.findOne(q4, true)->lookupDouble("x"));

  Query::Ptr q5 = coll.createQuery();
  q5->append("name", "barbar");
  EXPECT_THROW(coll.findOne(q5, true), NoMatchingMessageException);
  EXPECT_THROW(coll.findOne(q5, false), NoMatchingMessageException);

  // Test update
  Metadata::Ptr m1 = coll.createMetadata();
  m1->append("name", "barbar");
  coll.modifyMetadata(q4, m1);
  EXPECT_EQ(3u, coll.count());
  EXPECT_THROW(coll.findOne(q4, false), NoMatchingMessageException);
  EXPECT_EQ(p1, *coll.findOne(q5, false));

  // Check stored metadata
  EXPECT_EQ("geometry_msgs/Pose", conn.messageType("my_db", "poses"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "client_test");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// %EndTag(CPP_CLIENT)%
