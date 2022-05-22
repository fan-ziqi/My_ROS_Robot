/*
 * filter_criteria.cpp
 *
 *  Created on: Jul 6, 2014
 *      Author: jorge
 */

#include <unique_id/unique_id.h>

#include "world_canvas_client_cpp/filter_criteria.hpp"

namespace wcf
{

FilterCriteria::FilterCriteria(const std::string & world)
{
  this->setWorld(world);
}

FilterCriteria::FilterCriteria(const             std::string & world,
                               const std::vector<std::string>& uuids,
                               const std::vector<std::string>& names,
                               const std::vector<std::string>& types,
                               const std::vector<std::string>& keywords,
                               const std::vector<std::string>& relationships)
{
  this->setWorld(world);
  this->setUuids(uuids);
  this->setNames(names);
  this->setTypes(types);
  this->setKeywords(keywords);
  this->setRelationships(relationships);
}

FilterCriteria::~FilterCriteria()
{
}

bool FilterCriteria::nullFilter()
{
  return uuids.size() == names.size() == types.size()
      == keywords.size() == relationships.size() == 0;
}

void FilterCriteria::setWorld(const std::string& world)
{
  this->world = world;
}

void FilterCriteria::setUuids(const std::vector<std::string>& uuids)
{
  this->uuids.clear();
  for (unsigned int i = 0; i < uuids.size(); i++)
    this->uuids.push_back(unique_id::toMsg(unique_id::fromHexString(uuids[i])));
}

void FilterCriteria::setNames(const std::vector<std::string>& names)
{
  this->names = names;
}

void FilterCriteria::setTypes(const std::vector<std::string>& types)
{
  this->types = types;
}

void FilterCriteria::setKeywords(const std::vector<std::string>& keywords)
{
  this->keywords = keywords;
}

void FilterCriteria::setRelationships(const std::vector<std::string>& relationships)
{
  this->relationships.clear();
  for (unsigned int i = 0; i < relationships.size(); i++)
    this->relationships.push_back(unique_id::toMsg(unique_id::fromHexString(relationships[i])));
}

} // namespace wcf
