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
 * Defines the TransformCollection class
 *
 * \author Bhaskara Marthi
 */

#ifndef WAREHOUSE_ROS_TF_COLLECTION_H
#define WAREHOUSE_ROS_TF_COLLECTION_H

#include <warehouse_ros/message_collection.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>

namespace warehouse_ros
{
/// This abstract base class just makes it easier to write code that works for
/// both TransformCollection and regular tf TransformListener objects
class TransformSource
{
public:
  /// Get the transform between two frames at a given timepoint.  Can throw
  /// all the usual tf exceptions if the transform is unavailable.
  virtual tf::StampedTransform lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                               double t) const = 0;
};

/// The setup is that you have a db containing a collection with tf messages,
/// in which each message has a metadata field named 'stamp', that equals the
/// tf timestamp (this could be generated, e.g., with bag_to_db followed by
/// add_metadata).
/// Given such a collection, this class allows querying for a transform as
/// with tf::TransformListener::lookupTransform, except this is deterministic
/// with no dependency on network state or message queues.
class TransformCollection : public TransformSource
{
public:
  TransformCollection(MessageCollection<tf::tfMessage>& coll, const double search_back = 10.0,
                      const double search_forward = 1.0)
    : TransformSource(), coll_(coll), search_back_(search_back), search_forward_(search_forward)
  {
  }

  /// Get the transform between two frames at a given timepoint.  Can throw
  /// all the exceptions tf::lookupTransform can.
  tf::StampedTransform lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                       double t) const override;

  /// Put the transform into the collection.
  void putTransform(tf::StampedTransform);

private:
  MessageCollection<tf::tfMessage> coll_;
  double search_back_;
  double search_forward_;
};

/// This wraps a tf transform listener so it can be used interchangeably
/// with a TransformCollection.
class LiveTransformSource : public TransformSource
{
public:
  /// \param timeout: Maximum timeout
  ///
  /// ros::init must be called before creating an instance
  LiveTransformSource(double timeout = 0) : TransformSource(), tf_(new tf::TransformListener()), timeout_(timeout)
  {
  }

  /// Will return the transform if it becomes available before the timeout
  /// expires, else throw a tf exception
  tf::StampedTransform lookupTransform(const std::string& target, const std::string& source, double t) const override
  {
    ros::Time tm(t);
    tf_->waitForTransform(target, source, tm, ros::Duration(timeout_));
    tf::StampedTransform trans;
    tf_->lookupTransform(target, source, tm, trans);  // Can throw
    return trans;
  }

private:
  ros::NodeHandle nh_;
  boost::shared_ptr<tf::TransformListener> tf_;
  ros::Duration timeout_;
};

}  // namespace

#endif  // include guard
