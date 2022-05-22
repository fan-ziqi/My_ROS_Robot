/*
 * Copyright (c) 2014, Savioke, Inc.
 * Copyright (c) 2016, Orbbec Ltd.
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
 *      Author: Tim Liu (liuhua@orbbec.com)
 */

/**
 * Small executable that creates a device manager to print the information of all devices including their
 * serial number.
 */

#include <iostream>
#include "astra_camera/astra_device_manager.h"
#include "astra_camera/astra_exception.h"

using astra_wrapper::AstraDeviceManager;
using astra_wrapper::AstraDeviceInfo;
using astra_wrapper::AstraException;

int main(int arc, char** argv)
{
  astra_wrapper::AstraDeviceManager manager;
  boost::shared_ptr<std::vector<astra_wrapper::AstraDeviceInfo> > device_infos = manager.getConnectedDeviceInfos();
  std::cout << "Found " << device_infos->size() << " devices:" << std::endl << std::endl;
  for (size_t i = 0; i < device_infos->size(); ++i)
  {
    std::cout << "Device #" << i << ":" << std::endl;
    std::cout << device_infos->at(i) << std::endl;
    try {
      std::string serial = manager.getSerial(device_infos->at(i).uri_);
      std::cout << "Serial number: " << serial << std::endl;
    }
    catch (const AstraException& exception)
    {
      std::cerr << "Could not retrieve serial number: " << exception.what() << std::endl;
    }
  }
  return 0;
}

