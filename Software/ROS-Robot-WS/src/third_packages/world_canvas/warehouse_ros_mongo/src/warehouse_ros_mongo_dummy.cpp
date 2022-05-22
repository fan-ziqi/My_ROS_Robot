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

#include <warehouse_ros_mongo/metadata.h>

// add this dummy function so the .so file copies everything we need from the
// libmongoclient.a file at link time. We need this because Ubuntu does not install
// a .so file for libmongoclient and the wrappers we have in this lib are templated.
// make this function globally accessible so strip --strip-unneeded does not remove symbols
void _thisFunctionShouldNeverBeCalled_MakeWarehouseROSMongoIncludeTheSymbolsWeNeed_(void)
{
  mongo::DBClientConnection* conn = new mongo::DBClientConnection();
  mongo::GridFS* gfs = new mongo::GridFS(*conn, "");
  mongo::BSONObj q;
  mongo::GridFile f = gfs->findFile(q);
  f.write(std::cout);
  gfs->removeFile("");
  q = gfs->storeFile(NULL, 0, "");
  mongo::LT;
  mongo::GT;
  mongo::LTE;
  mongo::GTE;
  delete gfs;
  delete conn;
}
