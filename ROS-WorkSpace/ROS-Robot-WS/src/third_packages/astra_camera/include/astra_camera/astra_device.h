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

#ifndef ASTRA_DEVICE_H
#define ASTRA_DEVICE_H

#include "astra_camera/astra_video_mode.h"

#include "astra_camera/astra_exception.h"
#include "astra_camera/astra_device_type.h"

#include <openni2/OpenNI.h>

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <sensor_msgs/Image.h>

#include <string>
#include <vector>

namespace openni
{
class Device;
class DeviceInfo;
class VideoStream;
class SensorInfo;
}

namespace astra_wrapper
{

typedef boost::function<void(sensor_msgs::ImagePtr image)> FrameCallbackFunction;

class AstraFrameListener;

class AstraDevice
{
public:
  AstraDevice(const std::string& device_URI);
  virtual ~AstraDevice();

  const std::string getUri() const;
  const std::string getVendor() const;
  const std::string getName() const;
  uint16_t getUsbVendorId() const;
  uint16_t getUsbProductId() const;

  const std::string getStringID() const;

  bool isValid() const;

  bool hasIRSensor() const;
  bool hasColorSensor() const;
  bool hasDepthSensor() const;

  void startIRStream();
  void startColorStream();
  void startDepthStream();

  void stopAllStreams();

  void stopIRStream();
  void stopColorStream();
  void stopDepthStream();

  bool isIRStreamStarted();
  bool isColorStreamStarted();
  bool isDepthStreamStarted();

  bool isImageRegistrationModeSupported() const;
  void setImageRegistrationMode(bool enabled);
  void setDepthColorSync(bool enabled);

  const AstraVideoMode getIRVideoMode();
  const AstraVideoMode getColorVideoMode();
  const AstraVideoMode getDepthVideoMode();

  const std::vector<AstraVideoMode>& getSupportedIRVideoModes() const;
  const std::vector<AstraVideoMode>& getSupportedColorVideoModes() const;
  const std::vector<AstraVideoMode>& getSupportedDepthVideoModes() const;

  bool isIRVideoModeSupported(const AstraVideoMode& video_mode) const;
  bool isColorVideoModeSupported(const AstraVideoMode& video_mode) const;
  bool isDepthVideoModeSupported(const AstraVideoMode& video_mode) const;

  void setIRVideoMode(const AstraVideoMode& video_mode);
  void setColorVideoMode(const AstraVideoMode& video_mode);
  void setDepthVideoMode(const AstraVideoMode& video_mode);

  void setIRFrameCallback(FrameCallbackFunction callback);
  void setColorFrameCallback(FrameCallbackFunction callback);
  void setDepthFrameCallback(FrameCallbackFunction callback);

  float getIRFocalLength (int output_y_resolution) const;
  float getColorFocalLength (int output_y_resolution) const;
  float getDepthFocalLength (int output_y_resolution) const;
  float getBaseline () const;
  OBCameraParams getCameraParams() const;
  bool isCameraParamsValid();
  char* getSerialNumber();
  char* getDeviceType();
  OB_DEVICE_NO getDeviceTypeNo();
  int getIRGain() const;
  int getIRExposure() const;

  void setCameraParams(OBCameraParams param);
  void setIRGain(int gain);
  void setIRExposure(int exposure);
  void setLaser(bool enable);
  void setIRFlood(bool enable);
  void setLDP(bool enable);

  void switchIRCamera(int cam);

  void setAutoExposure(bool enable);
  void setAutoWhiteBalance(bool enable);

  bool getAutoExposure() const;
  bool getAutoWhiteBalance() const;

  void setUseDeviceTimer(bool enable);

protected:
  void shutdown();

  boost::shared_ptr<openni::VideoStream> getIRVideoStream() const;
  boost::shared_ptr<openni::VideoStream> getColorVideoStream() const;
  boost::shared_ptr<openni::VideoStream> getDepthVideoStream() const;

  boost::shared_ptr<openni::Device> openni_device_;
  boost::shared_ptr<openni::DeviceInfo> device_info_;

  boost::shared_ptr<AstraFrameListener> ir_frame_listener;
  boost::shared_ptr<AstraFrameListener> color_frame_listener;
  boost::shared_ptr<AstraFrameListener> depth_frame_listener;

  mutable boost::shared_ptr<openni::VideoStream> ir_video_stream_;
  mutable boost::shared_ptr<openni::VideoStream> color_video_stream_;
  mutable boost::shared_ptr<openni::VideoStream> depth_video_stream_;

  mutable std::vector<AstraVideoMode> ir_video_modes_;
  mutable std::vector<AstraVideoMode> color_video_modes_;
  mutable std::vector<AstraVideoMode> depth_video_modes_;

  bool ir_video_started_;
  bool color_video_started_;
  bool depth_video_started_;

  bool image_registration_activated_;

  bool use_device_time_;

  OBCameraParams m_CamParams;
  bool m_ParamsValid;
  char serial_number[12];
  char device_type[32];
  OB_DEVICE_NO device_type_no;
};

std::ostream& operator << (std::ostream& stream, const AstraDevice& device);

}

#endif /* OPENNI_DEVICE_H */
