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
 * Implementation of template methods of MessageCollection
 * Only to be included by message_collection.h
 *
 * \author Bhaskara Marthi
 */

namespace warehouse_ros
{
template <class M>
MessageCollection<M>::MessageCollection(MessageCollectionHelper::Ptr collection) : collection_(collection)
{
  typedef typename ros::message_traits::DataType<M> DataType;
  const std::string datatype = DataType().value();
  typedef typename ros::message_traits::MD5Sum<M> Md5;
  const std::string md5 = Md5().value();
  md5sum_matches_ = collection_->initialize(datatype, md5);
}

template <class M>
MessageCollection<M>::MessageCollection(const MessageCollection<M>& other)
  : collection_(other.collection_), md5sum_matches_(other.md5sum_matches_)
{
}

template <class M>
MessageCollection<M>::~MessageCollection()
{
}

template <class M>
MessageCollection<M>& MessageCollection<M>::operator=(const MessageCollection& other)
{
  collection_ = other.collection_;
  md5sum_matches_ = other.md5sum_matches_;
  return *this;
}

template <class M>
void MessageCollection<M>::insert(const M& msg, Metadata::Ptr metadata)
{
  if (!md5sum_matches_)
    throw Md5SumException("Cannot insert additional elements.");

  metadata->append("creation_time", ros::Time::now().toSec());

  /// Serialize the message into a buffer
  size_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, msg);
  char* data = (char*)buffer.get();

  collection_->insert(data, serial_size, metadata);
}

template <class M>
typename QueryResults<M>::range_t MessageCollection<M>::query(Query::ConstPtr query, bool metadata_only,
                                                              const std::string& sort_by, bool ascending) const
{
  if (!md5sum_matches_ && !metadata_only)
    throw Md5SumException("Can only query metadata.");

  ResultIteratorHelper::Ptr results = collection_->query(query, sort_by, ascending);
  return typename QueryResults<M>::range_t(ResultIterator<M>(results, metadata_only), ResultIterator<M>());
}

template <class M>
std::vector<typename MessageWithMetadata<M>::ConstPtr> MessageCollection<M>::queryList(Query::ConstPtr query,
                                                                                       bool metadata_only,
                                                                                       const std::string& sort_by,
                                                                                       bool ascending) const
{
  typename QueryResults<M>::range_t res = this->query(query, metadata_only, sort_by, ascending);
  return std::vector<typename MessageWithMetadata<M>::ConstPtr>(res.first, res.second);
}

template <class M>
typename MessageWithMetadata<M>::ConstPtr MessageCollection<M>::findOne(Query::ConstPtr query,
                                                                        const bool metadata_only) const
{
  typename QueryResults<M>::range_t res = this->query(query, metadata_only);
  if (res.first == res.second)
    throw NoMatchingMessageException(collection_->collectionName());
  return *res.first;
}

template <class M>
unsigned MessageCollection<M>::removeMessages(Query::ConstPtr query)
{
  return collection_->removeMessages(query);
}

template <class M>
void MessageCollection<M>::modifyMetadata(Query::ConstPtr q, Metadata::ConstPtr m)
{
  collection_->modifyMetadata(q, m);
}

template <class M>
unsigned MessageCollection<M>::count()
{
  return collection_->count();
}

template <class M>
bool MessageCollection<M>::md5SumMatches() const
{
  return md5sum_matches_;
}

template <class M>
Query::Ptr MessageCollection<M>::createQuery() const
{
  return collection_->createQuery();
}

template <class M>
Metadata::Ptr MessageCollection<M>::createMetadata() const
{
  return collection_->createMetadata();
}

}  // namespace
