/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#ifndef ASTRA_DEVICE_MANAGER_H_
#define ASTRA_DEVICE_MANAGER_H_

// #include <boost/thread/mutex.hpp>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "ros2_astra_camera/astra_wrapper/astra_device_info.h"

namespace astra_camera {

class AstraDeviceListener;
class AstraDevice;

class AstraDeviceManager {
 public:
  AstraDeviceManager();
  virtual ~AstraDeviceManager();

  static std::shared_ptr<AstraDeviceManager> getSingelton();

  std::shared_ptr<std::vector<AstraDeviceInfo> > getConnectedDeviceInfos()
      const;
  std::shared_ptr<std::vector<std::string> > getConnectedDeviceURIs() const;
  std::size_t getNumOfConnectedDevices() const;

  std::shared_ptr<AstraDevice> getAnyDevice();
  std::shared_ptr<AstraDevice> getDevice(const std::string& device_URI);

  std::string getSerial(const std::string& device_URI) const;

 protected:
  std::shared_ptr<AstraDeviceListener> device_listener_;

  static std::shared_ptr<AstraDeviceManager> singelton_;
};

std::ostream& operator<<(std::ostream& stream,
                         const AstraDeviceManager& device_manager);

}  // namespace astra_camera

#endif  // ASTRA_DEVICE_MANAGER_H_
