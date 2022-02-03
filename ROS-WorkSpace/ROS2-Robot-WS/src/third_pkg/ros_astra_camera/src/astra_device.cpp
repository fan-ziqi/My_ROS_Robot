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

#include "openni2/OpenNI.h"

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>

#include "astra_camera/astra_driver.h"
#include "astra_camera/astra_device.h"
#include "astra_camera/astra_exception.h"
#include "astra_camera/astra_convert.h"
#include "astra_camera/astra_frame_listener.h"

#include "openni2/PS1080.h"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <string>
#include <cmath>

namespace astra_wrapper
{

AstraDevice::AstraDevice(const std::string& device_URI):
    openni_device_(),
    ir_video_started_(false),
    color_video_started_(false),
    depth_video_started_(false),
    image_registration_activated_(false),
    use_device_time_(false)
{
  openni::Status rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK)
    THROW_OPENNI_EXCEPTION("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());

  openni_device_ = boost::make_shared<openni::Device>();

  if (device_URI.length() > 0)
  {
    rc = openni_device_->open(device_URI.c_str());
  }
  else
  {
    rc = openni_device_->open(openni::ANY_DEVICE);
  }

  if (rc != openni::STATUS_OK)
    THROW_OPENNI_EXCEPTION("Device open failed\n%s\n", openni::OpenNI::getExtendedError());

  device_info_ = boost::make_shared<openni::DeviceInfo>();
  *device_info_ = openni_device_->getDeviceInfo();

  int param_size = sizeof(OBCameraParams);
  openni_device_->getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t*)&m_CamParams, &param_size);
  m_ParamsValid = true;
  if (std::isnan(m_CamParams.l_intr_p[0]) || std::isnan(m_CamParams.l_intr_p[1]) ||
      std::isnan(m_CamParams.l_intr_p[2]) || std::isnan(m_CamParams.l_intr_p[3]))
  {
    m_ParamsValid = false;
  }

  int serial_number_size = sizeof(serial_number);
  memset(serial_number, 0, serial_number_size);
  openni_device_->getProperty(openni::OBEXTENSION_ID_SERIALNUMBER, (uint8_t*)&serial_number, &serial_number_size);

  int device_type_size = sizeof(device_type);
  memset(device_type, 0, device_type_size);
  openni_device_->getProperty(openni::OBEXTENSION_ID_DEVICETYPE, (uint8_t*)&device_type, &device_type_size);
  if (strcmp(device_type, OB_STEREO_S) == 0)
  {
    device_type_no = OB_STEREO_S_NO;
  }
  else if (strcmp(device_type, OB_EMBEDDED_S) == 0)
  {
    device_type_no = OB_EMBEDDED_S_NO;
  }
  else if (strcmp(device_type, OB_STEREO_S_U3) == 0)
  {
    device_type_no = OB_STEREO_S_U3_NO;
  }
  else if (strcmp(device_type, OB_ASTRA_PRO) == 0)
  {
    device_type_no = OB_ASTRA_PRO_NO;
  }
  else
  {
    device_type_no = OB_ASTRA_NO;
  }

  ir_frame_listener = boost::make_shared<AstraFrameListener>();
  color_frame_listener = boost::make_shared<AstraFrameListener>();
  depth_frame_listener = boost::make_shared<AstraFrameListener>();

}

AstraDevice::~AstraDevice()
{
  stopAllStreams();

  shutdown();

  openni_device_->close();
}

const std::string AstraDevice::getUri() const
{
  return std::string(device_info_->getUri());
}

const std::string AstraDevice::getVendor() const
{
  return std::string(device_info_->getVendor());
}

const std::string AstraDevice::getName() const
{
  return std::string(device_info_->getName());
}

uint16_t AstraDevice::getUsbVendorId() const
{
  return device_info_->getUsbVendorId();
}

uint16_t AstraDevice::getUsbProductId() const
{
  return device_info_->getUsbProductId();
}

const std::string AstraDevice::getStringID() const
{
  std::string ID_str = getName() + "_" + getVendor();

  boost::replace_all(ID_str, "/", "");
  boost::replace_all(ID_str, ".", "");
  boost::replace_all(ID_str, "@", "");

  return ID_str;
}

bool AstraDevice::isValid() const
{
  return (openni_device_.get() != 0) && openni_device_->isValid();
}

OBCameraParams AstraDevice::getCameraParams() const
{
  return m_CamParams;
}

bool AstraDevice::isCameraParamsValid()
{
  return m_ParamsValid;
}

char* AstraDevice::getSerialNumber()
{
  return serial_number;
}

char* AstraDevice::getDeviceType()
{
  return device_type;
}

OB_DEVICE_NO AstraDevice::getDeviceTypeNo()
{
  return device_type_no;
}

int AstraDevice::getIRGain() const
{
  int gain = 0;
  int data_size = 4;
  openni_device_->getProperty(openni::OBEXTENSION_ID_IR_GAIN, (uint8_t*)&gain, &data_size);
  return gain;
}

int AstraDevice::getIRExposure() const
{
  int exposure = 0;
  int data_size = 4;
  openni_device_->getProperty(openni::OBEXTENSION_ID_IR_EXP, (uint8_t*)&exposure, &data_size);
  return exposure;
}

void AstraDevice::setCameraParams(OBCameraParams param)
{
  int data_size = sizeof(OBCameraParams);
  openni_device_->setProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t*)&param, data_size);
}

void AstraDevice::setIRGain(int gain)
{
  int data_size = 4;
  openni_device_->setProperty(openni::OBEXTENSION_ID_IR_GAIN, (uint8_t*)&gain, data_size);
}

void AstraDevice::setIRExposure(int exposure)
{
  int data_size = 4;
  openni_device_->setProperty(openni::OBEXTENSION_ID_IR_EXP, (uint8_t*)&exposure, data_size);
}

void AstraDevice::setLaser(bool enable)
{
  int data_size = 4;
  int laser_enable = 1;
  if (enable == false)
  {
    laser_enable = 0;
  }
  openni_device_->setProperty(openni::OBEXTENSION_ID_LASER_EN, (uint8_t*)&laser_enable, data_size);
}

void AstraDevice::setIRFlood(bool enable)
{
  int data_size = 4;
  int enable_ = 1;
  if (enable == false)
  {
    enable_ = 0;
  }
  openni_device_->setProperty(XN_MODULE_PROPERTY_IRFLOOD_STATE, enable_);
}

void AstraDevice::setLDP(bool enable)
{
  int data_size = 4;
  int enable_ = 1;
  if (enable == false)
  {
    enable_ = 0;
  }
  if (device_type_no == OB_STEREO_S_U3_NO)
  {
    openni_device_->setProperty(XN_MODULE_PROPERTY_LDP_ENABLE, (uint8_t *)&enable_, 4);
  }
  else
  {
    boost::shared_ptr<openni::VideoStream> depth_stream = getDepthVideoStream();
    boost::shared_ptr<openni::VideoStream> ir_stream = getIRVideoStream();
    depth_stream->stop();
    ir_stream->stop();
    openni_device_->setProperty(openni::OBEXTENSION_ID_LDP_EN, (uint8_t *)&enable_, 4);
    depth_stream->start();
    ir_stream->start();
  }
}

void AstraDevice::switchIRCamera(int cam)
{
  if (device_type_no == OB_STEREO_S_NO || device_type_no == OB_STEREO_S_U3_NO)
  {
    openni_device_->setProperty(XN_MODULE_PROPERTY_SWITCH_IR, (uint8_t*)&cam, 4);
  }
}

float AstraDevice::getIRFocalLength(int output_y_resolution) const
{
  float focal_length = 0.0f;
  boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();

  if (stream)
  {
    focal_length = (float)output_y_resolution / (2 * tan(stream->getVerticalFieldOfView() / 2));
  }

  return focal_length;
}

float AstraDevice::getColorFocalLength(int output_y_resolution) const
{
  float focal_length = 0.0f;
  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    focal_length = (float)output_y_resolution / (2 * tan(stream->getVerticalFieldOfView() / 2));
  }

  return focal_length;
}

float AstraDevice::getDepthFocalLength(int output_y_resolution) const
{
  float focal_length = 0.0f;
  boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

  if (stream)
  {
    focal_length = (float)output_y_resolution / (2 * tan(stream->getVerticalFieldOfView() / 2));
  }

  return focal_length;
}

float AstraDevice::getBaseline() const
{
  return 0.075f;
}

bool AstraDevice::isIRVideoModeSupported(const AstraVideoMode& video_mode) const
{
  getSupportedIRVideoModes();

  bool supported = false;

  std::vector<AstraVideoMode>::const_iterator it = ir_video_modes_.begin();
  std::vector<AstraVideoMode>::const_iterator it_end = ir_video_modes_.end();

  while (it != it_end && !supported)
  {
    supported = (*it == video_mode);
    ++it;
  }

  return supported;
}

bool AstraDevice::isColorVideoModeSupported(const AstraVideoMode& video_mode) const
{
  getSupportedColorVideoModes();

  bool supported = false;

  std::vector<AstraVideoMode>::const_iterator it = color_video_modes_.begin();
  std::vector<AstraVideoMode>::const_iterator it_end = color_video_modes_.end();

  while (it != it_end && !supported)
  {
    supported = (*it == video_mode);
    ++it;
  }

  return supported;
}

bool AstraDevice::isDepthVideoModeSupported(const AstraVideoMode& video_mode) const
{
  getSupportedDepthVideoModes();

  bool supported = false;

  std::vector<AstraVideoMode>::const_iterator it = depth_video_modes_.begin();
  std::vector<AstraVideoMode>::const_iterator it_end = depth_video_modes_.end();

  while (it != it_end && !supported)
  {
    supported = (*it == video_mode);
    ++it;
  }

  return supported;

}

bool AstraDevice::hasIRSensor() const
{
  return openni_device_->hasSensor(openni::SENSOR_IR);
}

bool AstraDevice::hasColorSensor() const
{
  return (getUsbProductId()!=0x0403)?openni_device_->hasSensor(openni::SENSOR_COLOR):0;
}

bool AstraDevice::hasDepthSensor() const
{
  return openni_device_->hasSensor(openni::SENSOR_DEPTH);
}

void AstraDevice::startIRStream()
{
  boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();

  if (stream)
  {
    stream->setMirroringEnabled(false);
    stream->start();
    stream->addNewFrameListener(ir_frame_listener.get());
    ir_video_started_ = true;
  }

}

void AstraDevice::startColorStream()
{
  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    stream->setMirroringEnabled(false);
    stream->start();
    stream->addNewFrameListener(color_frame_listener.get());
    color_video_started_ = true;
  }
}
void AstraDevice::startDepthStream()
{
  boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

  if (stream)
  {
    stream->setMirroringEnabled(false);
    stream->start();
    stream->addNewFrameListener(depth_frame_listener.get());
    depth_video_started_ = true;
  }
}

void AstraDevice::stopAllStreams()
{
  stopIRStream();
  stopColorStream();
  stopDepthStream();
}

void AstraDevice::stopIRStream()
{
  if (ir_video_stream_.get() != 0)
  {
    ir_video_started_ = false;

    ir_video_stream_->removeNewFrameListener(ir_frame_listener.get());

    ir_video_stream_->stop();
  }
}
void AstraDevice::stopColorStream()
{
  if (color_video_stream_.get() != 0)
  {
    color_video_started_ = false;

    color_video_stream_->removeNewFrameListener(color_frame_listener.get());

    color_video_stream_->stop();
  }
}
void AstraDevice::stopDepthStream()
{
  if (depth_video_stream_.get() != 0)
  {
    depth_video_started_ = false;

    depth_video_stream_->removeNewFrameListener(depth_frame_listener.get());

    depth_video_stream_->stop();
  }
}

void AstraDevice::shutdown()
{
  if (ir_video_stream_.get() != 0)
    ir_video_stream_->destroy();

  if (color_video_stream_.get() != 0)
    color_video_stream_->destroy();

  if (depth_video_stream_.get() != 0)
    depth_video_stream_->destroy();

}

bool AstraDevice::isIRStreamStarted()
{
  return ir_video_started_;
}
bool AstraDevice::isColorStreamStarted()
{
  return color_video_started_;
}
bool AstraDevice::isDepthStreamStarted()
{
  return depth_video_started_;
}

const std::vector<AstraVideoMode>& AstraDevice::getSupportedIRVideoModes() const
{
  boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();

  ir_video_modes_.clear();

  if (stream)
  {
    const openni::SensorInfo& sensor_info = stream->getSensorInfo();

    ir_video_modes_ = astra_convert(sensor_info.getSupportedVideoModes());
  }

  return ir_video_modes_;
}

const std::vector<AstraVideoMode>& AstraDevice::getSupportedColorVideoModes() const
{
  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  color_video_modes_.clear();

  if (stream)
  {
    const openni::SensorInfo& sensor_info = stream->getSensorInfo();

    color_video_modes_ = astra_convert(sensor_info.getSupportedVideoModes());
  }

  return color_video_modes_;
}

const std::vector<AstraVideoMode>& AstraDevice::getSupportedDepthVideoModes() const
{
  boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

  depth_video_modes_.clear();

  if (stream)
  {
    const openni::SensorInfo& sensor_info = stream->getSensorInfo();

    depth_video_modes_ = astra_convert(sensor_info.getSupportedVideoModes());
  }

  return depth_video_modes_;
}

bool AstraDevice::isImageRegistrationModeSupported() const
{
  return openni_device_->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
}

void AstraDevice::setImageRegistrationMode(bool enabled)
{
  if (isImageRegistrationModeSupported())
  {
    image_registration_activated_ = enabled;
    if (enabled)
    {
      openni::Status rc = openni_device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
      if (rc != openni::STATUS_OK)
        THROW_OPENNI_EXCEPTION("Enabling image registration mode failed: \n%s\n", openni::OpenNI::getExtendedError());
    }
    else
    {
      openni::Status rc = openni_device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
      if (rc != openni::STATUS_OK)
        THROW_OPENNI_EXCEPTION("Enabling image registration mode failed: \n%s\n", openni::OpenNI::getExtendedError());
    }
  }
}

void AstraDevice::setDepthColorSync(bool enabled)
{
  openni::Status rc = openni_device_->setDepthColorSyncEnabled(enabled);
  if (rc != openni::STATUS_OK)
    THROW_OPENNI_EXCEPTION("Enabling depth color synchronization failed: \n%s\n", openni::OpenNI::getExtendedError());
}

const AstraVideoMode AstraDevice::getIRVideoMode()
{
  AstraVideoMode ret;

  boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();

  if (stream)
  {
    openni::VideoMode video_mode = stream->getVideoMode();

    ret = astra_convert(video_mode);
  }
  else
    THROW_OPENNI_EXCEPTION("Could not create video stream.");

  return ret;
}

const AstraVideoMode AstraDevice::getColorVideoMode()
{
  AstraVideoMode ret;

  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    openni::VideoMode video_mode = stream->getVideoMode();

    ret = astra_convert(video_mode);
  }
  else
    THROW_OPENNI_EXCEPTION("Could not create video stream.");

  return ret;
}

const AstraVideoMode AstraDevice::getDepthVideoMode()
{
  AstraVideoMode ret;

  boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

  if (stream)
  {
    openni::VideoMode video_mode = stream->getVideoMode();

    ret = astra_convert(video_mode);
  }
  else
    THROW_OPENNI_EXCEPTION("Could not create video stream.");

  return ret;
}

void AstraDevice::setIRVideoMode(const AstraVideoMode& video_mode)
{
  boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();

  if (stream)
  {
    const openni::VideoMode videoMode = astra_convert(video_mode);
    const openni::Status rc = stream->setVideoMode(videoMode);
    if (rc != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION("Couldn't set IR video mode: \n%s\n", openni::OpenNI::getExtendedError());
  }
}

void AstraDevice::setColorVideoMode(const AstraVideoMode& video_mode)
{
  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    const openni::VideoMode videoMode = astra_convert(video_mode);
    const openni::Status rc = stream->setVideoMode(videoMode);
    if (rc != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION("Couldn't set color video mode: \n%s\n", openni::OpenNI::getExtendedError());
  }
}

void AstraDevice::setDepthVideoMode(const AstraVideoMode& video_mode)
{
  boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

  if (stream)
  {
    const openni::VideoMode videoMode = astra_convert(video_mode);
    const openni::Status rc = stream->setVideoMode(videoMode);
    if (rc != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION("Couldn't set depth video mode: \n%s\n", openni::OpenNI::getExtendedError());
  }
}

void AstraDevice::setAutoExposure(bool enable)
{
  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    openni::CameraSettings* camera_seeting = stream->getCameraSettings();
    if (camera_seeting)
    {
      const openni::Status rc = camera_seeting->setAutoExposureEnabled(enable);
      if (rc != openni::STATUS_OK)
        THROW_OPENNI_EXCEPTION("Couldn't set auto exposure: \n%s\n", openni::OpenNI::getExtendedError());
    }

  }
}
void AstraDevice::setAutoWhiteBalance(bool enable)
{
  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    openni::CameraSettings* camera_seeting = stream->getCameraSettings();
    if (camera_seeting)
    {
      const openni::Status rc = camera_seeting->setAutoWhiteBalanceEnabled(enable);
      if (rc != openni::STATUS_OK)
        THROW_OPENNI_EXCEPTION("Couldn't set auto white balance: \n%s\n", openni::OpenNI::getExtendedError());
    }

  }
}

bool AstraDevice::getAutoExposure() const
{
  bool ret = false;

  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    openni::CameraSettings* camera_seeting = stream->getCameraSettings();
    if (camera_seeting)
      ret = camera_seeting->getAutoExposureEnabled();
  }

  return ret;
}
bool AstraDevice::getAutoWhiteBalance() const
{
  bool ret = false;

  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    openni::CameraSettings* camera_seeting = stream->getCameraSettings();
    if (camera_seeting)
      ret = camera_seeting->getAutoWhiteBalanceEnabled();
  }

  return ret;
}

void AstraDevice::setUseDeviceTimer(bool enable)
{
  if (ir_frame_listener)
    ir_frame_listener->setUseDeviceTimer(enable);

  if (color_frame_listener)
    color_frame_listener->setUseDeviceTimer(enable);

  if (depth_frame_listener)
    depth_frame_listener->setUseDeviceTimer(enable);
}

void AstraDevice::setIRFrameCallback(FrameCallbackFunction callback)
{
  ir_frame_listener->setCallback(callback);
}

void AstraDevice::setColorFrameCallback(FrameCallbackFunction callback)
{
  color_frame_listener->setCallback(callback);
}

void AstraDevice::setDepthFrameCallback(FrameCallbackFunction callback)
{
  depth_frame_listener->setCallback(callback);
}

boost::shared_ptr<openni::VideoStream> AstraDevice::getIRVideoStream() const
{
  if (ir_video_stream_.get() == 0)
  {
    if (hasIRSensor())
    {
      ir_video_stream_ = boost::make_shared<openni::VideoStream>();

      const openni::Status rc = ir_video_stream_->create(*openni_device_, openni::SENSOR_IR);
      if (rc != openni::STATUS_OK)
        THROW_OPENNI_EXCEPTION("Couldn't create IR video stream: \n%s\n", openni::OpenNI::getExtendedError());
    }
  }
  return ir_video_stream_;
}

boost::shared_ptr<openni::VideoStream> AstraDevice::getColorVideoStream() const
{
  if (color_video_stream_.get() == 0)
  {
    if (hasColorSensor())
    {
      color_video_stream_ = boost::make_shared<openni::VideoStream>();

      const openni::Status rc = color_video_stream_->create(*openni_device_, openni::SENSOR_COLOR);
      if (rc != openni::STATUS_OK)
        THROW_OPENNI_EXCEPTION("Couldn't create color video stream: \n%s\n", openni::OpenNI::getExtendedError());
    }
  }
  return color_video_stream_;
}

boost::shared_ptr<openni::VideoStream> AstraDevice::getDepthVideoStream() const
{
  if (depth_video_stream_.get() == 0)
  {
    if (hasDepthSensor())
    {
      depth_video_stream_ = boost::make_shared<openni::VideoStream>();

      const openni::Status rc = depth_video_stream_->create(*openni_device_, openni::SENSOR_DEPTH);
      if (rc != openni::STATUS_OK)
        THROW_OPENNI_EXCEPTION("Couldn't create depth video stream: \n%s\n", openni::OpenNI::getExtendedError());
    }
  }
  return depth_video_stream_;
}

std::ostream& operator <<(std::ostream& stream, const AstraDevice& device)
{

  stream << "Device info (" << device.getUri() << ")" << std::endl;
  stream << "   Vendor: " << device.getVendor() << std::endl;
  stream << "   Name: " << device.getName() << std::endl;
  stream << "   USB Vendor ID: " << device.getUsbVendorId() << std::endl;
  stream << "   USB Product ID: " << device.getUsbVendorId() << std::endl << std::endl;

  if (device.hasIRSensor())
  {
    stream << "IR sensor video modes:" << std::endl;
    const std::vector<AstraVideoMode>& video_modes = device.getSupportedIRVideoModes();

    std::vector<AstraVideoMode>::const_iterator it = video_modes.begin();
    std::vector<AstraVideoMode>::const_iterator it_end = video_modes.end();
    for (; it != it_end; ++it)
      stream << "   - " << *it << std::endl;
  }
  else
  {
    stream << "No IR sensor available" << std::endl;
  }

  if (device.hasColorSensor())
  {
    stream << "Color sensor video modes:" << std::endl;
    const std::vector<AstraVideoMode>& video_modes = device.getSupportedColorVideoModes();

    std::vector<AstraVideoMode>::const_iterator it = video_modes.begin();
    std::vector<AstraVideoMode>::const_iterator it_end = video_modes.end();
    for (; it != it_end; ++it)
      stream << "   - " << *it << std::endl;
  }
  else
  {
    stream << "No Color sensor available" << std::endl;
  }

  if (device.hasDepthSensor())
  {
    stream << "Depth sensor video modes:" << std::endl;
    const std::vector<AstraVideoMode>& video_modes = device.getSupportedDepthVideoModes();

    std::vector<AstraVideoMode>::const_iterator it = video_modes.begin();
    std::vector<AstraVideoMode>::const_iterator it_end = video_modes.end();
    for (; it != it_end; ++it)
      stream << "   - " << *it << std::endl;
  }
  else
  {
    stream << "No Depth sensor available" << std::endl;
  }

  return stream;
}

}
