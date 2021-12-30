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
 * Template implementation for ResultIterator.
 * Only to be included from query_results.h
 *
 * \author Bhaskara Marthi
 */

namespace warehouse_ros
{
template <class M>
ResultIterator<M>::ResultIterator(ResultIteratorHelper::Ptr results, bool metadata_only)
  : results_(results), metadata_only_(metadata_only)
{
  if (!results_->hasData())
    results_.reset();
}

template <class M>
ResultIterator<M>::ResultIterator(const ResultIterator<M>& other)
  : results_(other.results_), metadata_only_(other.metadata_only_)
{
}

template <class M>
ResultIterator<M>::ResultIterator() : metadata_only_(false)
{
}

template <class M>
ResultIterator<M>::~ResultIterator()
{
}

template <class M>
ResultIterator<M>& ResultIterator<M>::operator=(const ResultIterator& other)
{
  results_ = other.results_;
  metadata_only_ = other.metadata_only_;
  return *this;
}

template <class M>
void ResultIterator<M>::increment()
{
  if (!results_->next())
  {
    results_.reset();
  }
}

template <class M>
typename MessageWithMetadata<M>::ConstPtr ResultIterator<M>::dereference() const
{
  ROS_ASSERT(results_);

  typename MessageWithMetadata<M>::Ptr msg(new MessageWithMetadata<M>(results_->metadata()));
  if (!metadata_only_)
  {
    std::string str = results_->message();
    uint8_t* buf = (uint8_t*)str.c_str();
    ros::serialization::IStream istream(buf, str.size());
    ros::serialization::Serializer<M>::read(istream, *msg);
  }
  return msg;
}

template <class M>
bool ResultIterator<M>::equal(const ResultIterator<M>& other) const
{
  // Incomplete, the only case we care about is whether iter is at the end
  return (!results_ && !other.results_);
}

}  // namespace
