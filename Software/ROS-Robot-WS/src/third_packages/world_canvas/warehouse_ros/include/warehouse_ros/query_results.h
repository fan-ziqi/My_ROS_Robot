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
 * Defines an iterator type over results of a query
 *
 * \author Bhaskara Marthi
 */

#ifndef WAREHOUSE_ROS_QUERY_RESULTS_H
#define WAREHOUSE_ROS_QUERY_RESULTS_H

#include <warehouse_ros/message_with_metadata.h>
#include <warehouse_ros/exceptions.h>
#include <boost/iterator/iterator_facade.hpp>

namespace warehouse_ros
{
class ResultIteratorHelper
{
public:
  virtual bool next() = 0;
  virtual bool hasData() const = 0;
  virtual Metadata::ConstPtr metadata() const = 0;
  virtual std::string message() const = 0;

  typedef boost::shared_ptr<ResultIteratorHelper> Ptr;
};

template <class M>
class ResultIterator
    : public boost::iterator_facade<ResultIterator<M>, typename MessageWithMetadata<M>::ConstPtr,
                                    boost::single_pass_traversal_tag, typename MessageWithMetadata<M>::ConstPtr>
{
public:
  /// \brief Constructor
  ResultIterator(ResultIteratorHelper::Ptr results, bool metadata_only);

  /// \brief Copy constructor
  ResultIterator(const ResultIterator& rhs);

  /// \brief Constructor for past_the_end iterator
  ResultIterator();

  /// \brief Destructor
  ~ResultIterator();

  ResultIterator& operator=(const ResultIterator& other);

private:
  friend class boost::iterator_core_access;

  // Member functions needed to be an iterator
  void increment();
  typename MessageWithMetadata<M>::ConstPtr dereference() const;
  bool equal(const ResultIterator<M>& other) const;

  ResultIteratorHelper::Ptr results_;
  const bool metadata_only_;
};

template <class M>
struct QueryResults
{
  typedef std::pair<ResultIterator<M>, ResultIterator<M> > range_t;
};

}  // namespace

#include "impl/query_results_impl.hpp"

#endif  // include guard
