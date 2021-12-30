/*
 * world_collection.hpp
 *
 *  Created on: Oct 13, 2014
 *      Author: jorge
 */

#ifndef WORLD_COLLECTION_HPP_
#define WORLD_COLLECTION_HPP_

#include "world_canvas_client_cpp/world_canvas_client.hpp"

namespace wcf
{

/**
 * Manages the collection of worlds currently on database.
 * The collection is loaded at startup.
 */
class WorldCollection : public WorldCanvasClient
{
protected:
  std::vector<std::string> world_names;

public:
  /**
   * Initializes the collection of worlds and fills it by calling world canvas server.
   *
   * @param srv_namespace: World canvas handles can be found under this namespace.
   */
  WorldCollection(const std::string& srv_namespace = "");
};

} // namespace wcf

#endif /* WORLD_COLLECTION_HPP_ */
