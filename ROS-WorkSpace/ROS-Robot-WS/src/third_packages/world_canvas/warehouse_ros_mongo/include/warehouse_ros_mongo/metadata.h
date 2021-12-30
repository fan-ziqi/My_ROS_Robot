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
 * Define a couple of classes that wrap Mongo's BSON type
 *
 * \author Bhaskara Marthi
 */

#ifndef WAREHOUSE_ROS_MONGO_METADATA_H
#define WAREHOUSE_ROS_MONGO_METADATA_H

// We have to include this top-level include here because
// the mongo c++ library is not robust to reincludes
#ifdef __APPLE__
#include <malloc/malloc.h>
#else
#include <malloc.h>
#endif
#include <warehouse_ros/metadata.h>
#include <warehouse_ros_mongo/config.h>

#include <mongo/db/json.h>

namespace warehouse_ros_mongo
{
using mongo::BSONObj;
using mongo::BSONObjBuilder;

/// \brief Internal parent class
///
/// This allows the user to not have to deal with separate BSONObj and
/// BSONObj builder objects
class WrappedBSON : public BSONObj
{
public:
  WrappedBSON() : BSONObj(), builder_(new BSONObjBuilder())
  {
  }

  WrappedBSON(const WrappedBSON& other) : BSONObj(), builder_(other.builder_)
  {
    update();
  }

  WrappedBSON(const BSONObj& other) : BSONObj(), builder_(new BSONObjBuilder())
  {
    builder_->appendElements(other);
    update();
  }

  WrappedBSON(const std::string& json) : BSONObj(), builder_(new BSONObjBuilder())
  {
    builder_->appendElements(mongo::fromjson(json.c_str()));
    update();
  }

protected:
  boost::shared_ptr<BSONObjBuilder> builder_;

  void update()
  {
    BSONObj::operator=(builder_->asTempObj());
  }
};

/// \brief Represents a query to the db
///
/// Usage:
/// Query q("foo", 42);
/// Query q2("bar", LT, 24); // bar less than 24
/// Templated so you can have different types of values
///
/// Or:
/// q = Query().append("foo", 42).append("bar", LT, 24);
class MongoQuery : public WrappedBSON, public warehouse_ros::Query
{
public:
  MongoQuery() : WrappedBSON()
  {
  }

  MongoQuery(const MongoQuery& other) : WrappedBSON(other)
  {
  }

  MongoQuery(const BSONObj& other) : WrappedBSON(other)
  {
  }

  void append(const std::string& name, const std::string& val)
  {
    *builder_ << name << val;
    WrappedBSON::update();
  }

  void append(const std::string& name, const double val)
  {
    *builder_ << name << val;
    WrappedBSON::update();
  }

  void append(const std::string& name, const int val)
  {
    *builder_ << name << val;
    WrappedBSON::update();
  }

  void append(const std::string& name, const bool val)
  {
    *builder_ << name << val;
    WrappedBSON::update();
  }

  void appendLT(const std::string& name, const double val)
  {
    *builder_ << name << mongo::LT << val;
    WrappedBSON::update();
  }

  void appendLT(const std::string& name, const int val)
  {
    *builder_ << name << mongo::LT << val;
    WrappedBSON::update();
  }

  void appendLTE(const std::string& name, const double val)
  {
    *builder_ << name << mongo::LTE << val;
    WrappedBSON::update();
  }

  void appendLTE(const std::string& name, const int val)
  {
    *builder_ << name << mongo::LTE << val;
    WrappedBSON::update();
  }

  void appendGT(const std::string& name, const double val)
  {
    *builder_ << name << mongo::GT << val;
    WrappedBSON::update();
  }

  void appendGT(const std::string& name, const int val)
  {
    *builder_ << name << mongo::GT << val;
    WrappedBSON::update();
  }

  void appendGTE(const std::string& name, const double val)
  {
    *builder_ << name << mongo::GTE << val;
    WrappedBSON::update();
  }

  void appendGTE(const std::string& name, const int val)
  {
    *builder_ << name << mongo::GTE << val;
    WrappedBSON::update();
  }

  void appendRange(const std::string& name, const double lower, const double upper)
  {
    *builder_ << name << mongo::GT << lower << mongo::LT << upper;
    WrappedBSON::update();
  }

  void appendRange(const std::string& name, const int lower, const int upper)
  {
    *builder_ << name << mongo::GT << lower << mongo::LT << upper;
    WrappedBSON::update();
  }

  void appendRangeInclusive(const std::string& name, const double lower, const double upper)
  {
    *builder_ << name << mongo::GTE << lower << mongo::LTE << upper;
    WrappedBSON::update();
  }

  void appendRangeInclusive(const std::string& name, const int lower, const int upper)
  {
    *builder_ << name << mongo::GTE << lower << mongo::LTE << upper;
    WrappedBSON::update();
  }
};

/// \brief Represents metadata attached to a message.  Automatically
/// includes a unique id and creation time.
///
/// Usage:
///
/// Metadata m("x", 24, "y", 42);
/// (templated so you can use varying number of fields, numeric or string values)
///
/// Or:
/// m = Metadata().append("x", 24).append("name", "foo");
class MongoMetadata : public warehouse_ros::Metadata, public WrappedBSON
{
public:
  MongoMetadata() : WrappedBSON()
  {
    initialize();
  }

  MongoMetadata(const std::string& json) : WrappedBSON(json)
  {
  }

  MongoMetadata(const MongoMetadata& other) : WrappedBSON(other)
  {
  }

  MongoMetadata(const BSONObj& other) : WrappedBSON(other)
  {
  }

  void append(const std::string& name, const std::string& val)
  {
    *builder_ << name << val;
    WrappedBSON::update();
  }

  void append(const std::string& name, const double val)
  {
    *builder_ << name << val;
    WrappedBSON::update();
  }

  void append(const std::string& name, const int val)
  {
    *builder_ << name << val;
    WrappedBSON::update();
  }

  void append(const std::string& name, const bool val)
  {
    *builder_ << name << val;
    WrappedBSON::update();
  }

  std::string lookupString(const std::string& name) const
  {
    return getStringField(name.c_str());
  }

  double lookupDouble(const std::string& name) const
  {
    double d;
    (*this)[name.c_str()].Val(d);
    return d;
  }

  int lookupInt(const std::string& name) const
  {
    return getIntField(name.c_str());
  }

  bool lookupBool(const std::string& name) const
  {
    return getBoolField(name.c_str());
  }

  bool lookupField(const std::string& name) const
  {
    return BSONObj::hasField(name.c_str());
  }

  std::set<std::string> lookupFieldNames() const
  {
    std::set<std::string> fields;
    BSONObj::getFieldNames(fields);
    return fields;
  }

private:
  void initialize()
  {
    builder_->genOID();
    WrappedBSON::update();
  }
};

}  // namespace

#endif  // include guard
