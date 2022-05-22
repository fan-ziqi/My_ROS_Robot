/*
 * filter_criteria.hpp
 *
 *  Created on: Jul 6, 2014
 *      Author: jorge
 */

#ifndef FILTER_CRITERIA_HPP_
#define FILTER_CRITERIA_HPP_

#include <string>
#include <vector>

#include <uuid_msgs/UniqueID.h>

typedef uuid_msgs::UniqueID UniqueIDmsg;

namespace wcf
{

/**
 * Annotations filter criteria to pass to the world canvas server. The only
 * mandatory criteria is the world name.
 * Empty vectors are ignored; the non-empty are concatenated with logic ANDs.
 * Within the lists, elements are concatenated with logic ORs, so for example
 * an annotation only needs to contain one of the keywords to be retrieved.
 */
class FilterCriteria
{
private:
  std::string              world;
  std::vector<UniqueIDmsg> uuids;
  std::vector<std::string> names;
  std::vector<std::string> types;
  std::vector<std::string> keywords;
  std::vector<UniqueIDmsg> relationships;

public:
  /**
   * Creates an empty filter criteria set.
   */
  FilterCriteria(const std::string& world);

  /**
   * Creates a filter criteria set at one blow.
   *
   * @param uuids:         Filter annotations by their uuid
   * @param names:         Filter annotations by their name
   * @param types:         Filter annotations by their type
   * @param keywords:      Filter annotations by their keywords
   * @param relationships: Filter annotations by their relationships
   */
  FilterCriteria(const std::string & world,
                 const std::vector<std::string>& uuids,
                 const std::vector<std::string>& names,
                 const std::vector<std::string>& types,
                 const std::vector<std::string>& keywords,
                 const std::vector<std::string>& relationships);

  virtual ~FilterCriteria();

  bool nullFilter();

  void setWorld(const std::string& world);
  void setUuids(const std::vector<std::string>& uuids);
  void setNames(const std::vector<std::string>& names);
  void setTypes(const std::vector<std::string>& types);
  void setKeywords(const std::vector<std::string>& keywords);
  void setRelationships(const std::vector<std::string>& relationships);

              std::string  getWorld() const { return world; }
  std::vector<UniqueIDmsg> getUuids() const { return uuids; }
  std::vector<std::string> getNames() const { return names; }
  std::vector<std::string> getTypes() const { return types; }
  std::vector<std::string> getKeywords() const { return keywords; }
  std::vector<UniqueIDmsg> getRelationships() const { return relationships; }
};

} // namespace wcf

#endif /* FILTER_CRITERIA_HPP_ */
