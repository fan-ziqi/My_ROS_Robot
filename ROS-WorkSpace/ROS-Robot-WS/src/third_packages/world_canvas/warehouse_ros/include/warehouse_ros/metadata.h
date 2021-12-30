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
 * Define a couple of classes for wrapping queries and metadata
 *
 * \author Bhaskara Marthi
 */

#ifndef WAREHOUSE_ROS_METADATA_H
#define WAREHOUSE_ROS_METADATA_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <set>

namespace warehouse_ros
{
/// \brief Represents a query to the db
///
/// Usage:
/// q = Query().append("foo", 42).appendLT("bar", 24);
class Query
{
public:
  typedef boost::shared_ptr<Query> Ptr;
  typedef boost::shared_ptr<const Query> ConstPtr;

  virtual ~Query()
  {
  }
  virtual void append(const std::string& name, const std::string& val) = 0;
  virtual void append(const std::string& name, const double val) = 0;
  virtual void append(const std::string& name, const int val) = 0;
  virtual void append(const std::string& name, const bool val) = 0;
  virtual void appendLT(const std::string& name, const double val) = 0;
  virtual void appendLT(const std::string& name, const int val) = 0;
  virtual void appendLTE(const std::string& name, const double val) = 0;
  virtual void appendLTE(const std::string& name, const int val) = 0;
  virtual void appendGT(const std::string& name, const double val) = 0;
  virtual void appendGT(const std::string& name, const int val) = 0;
  virtual void appendGTE(const std::string& name, const double val) = 0;
  virtual void appendGTE(const std::string& name, const int val) = 0;
  virtual void appendRange(const std::string& name, const double lower, const double upper) = 0;
  virtual void appendRange(const std::string& name, const int lower, const int upper) = 0;
  virtual void appendRangeInclusive(const std::string& name, const double lower, const double upper) = 0;
  virtual void appendRangeInclusive(const std::string& name, const int lower, const int upper) = 0;
};

/// \brief Represents metadata attached to a message.
///
/// Usage:
/// m = Metadata().append("x", 24).append("name", "foo");
class Metadata
{
public:
  typedef boost::shared_ptr<Metadata> Ptr;
  typedef boost::shared_ptr<const Metadata> ConstPtr;

  virtual ~Metadata()
  {
  }
  virtual void append(const std::string& name, const std::string& val) = 0;
  virtual void append(const std::string& name, const double val) = 0;
  virtual void append(const std::string& name, const int val) = 0;
  virtual void append(const std::string& name, const bool val) = 0;
  virtual std::string lookupString(const std::string& name) const = 0;
  virtual double lookupDouble(const std::string& name) const = 0;
  virtual int lookupInt(const std::string& name) const = 0;
  virtual bool lookupBool(const std::string& name) const = 0;
  virtual bool lookupField(const std::string& name) const = 0;
  virtual std::set<std::string> lookupFieldNames() const = 0;
};

}  // namespace

#endif  // include guard
