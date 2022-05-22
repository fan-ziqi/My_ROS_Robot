/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2012 Ken Tossell
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include "libuvc_camera/camera_driver.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <libuvc/libuvc.h>
#include <astra_camera/GetDeviceType.h>
#include <astra_camera/GetCameraInfo.h>
#include <cmath>

#define libuvc_VERSION (libuvc_VERSION_MAJOR * 10000 \
                      + libuvc_VERSION_MINOR * 100 \
                      + libuvc_VERSION_PATCH)

namespace libuvc_camera {

CameraDriver::CameraDriver(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  : nh_(nh), priv_nh_(priv_nh),
    state_(kInitial),
    ctx_(NULL), dev_(NULL), devh_(NULL), rgb_frame_(NULL),
    it_(nh_),
    config_server_(mutex_, priv_nh_),
    config_changed_(false),
    cinfo_manager_(nh),
    param_init_(false) {
  cam_pub_ = it_.advertiseCamera("image_raw", 1, false);
  ns = ros::this_node::getNamespace();
  device_type_client = nh_.serviceClient<astra_camera::GetDeviceType>(ns + "/get_device_type");
  camera_info_client = nh_.serviceClient<astra_camera::GetCameraInfo>(ns + "/get_camera_info");
  get_uvc_exposure_server = nh_.advertiseService("get_uvc_exposure", &CameraDriver::getUVCExposureCb, this);
  set_uvc_exposure_server = nh_.advertiseService("set_uvc_exposure", &CameraDriver::setUVCExposureCb, this);
  get_uvc_gain_server = nh_.advertiseService("get_uvc_gain", &CameraDriver::getUVCGainCb, this);
  set_uvc_gain_server = nh_.advertiseService("set_uvc_gain", &CameraDriver::setUVCGainCb, this);
  get_uvc_white_balance_server = nh_.advertiseService("get_uvc_white_balance", &CameraDriver::getUVCWhiteBalanceCb, this);
  set_uvc_white_balance_server = nh_.advertiseService("set_uvc_white_balance", &CameraDriver::setUVCWhiteBalanceCb, this);
  device_type_init_ = false;
  camera_info_init_ = false;
  uvc_flip_ = 0;
  device_type_no_ = OB_ASTRA_NO;
  int slash_end;
  for (slash_end = 0; slash_end < ns.length(); slash_end++)
  {
    if (ns[slash_end] != '/')
    {
      break;
    }
  }
  ns_no_slash = ns.substr(slash_end);
  camera_info_valid_ = false;
}

CameraDriver::~CameraDriver() {
  if (rgb_frame_)
    uvc_free_frame(rgb_frame_);

  if (ctx_)
    uvc_exit(ctx_);  // Destroys dev_, devh_, etc.
}

bool CameraDriver::getUVCExposureCb(astra_camera::GetUVCExposureRequest& req, astra_camera::GetUVCExposureResponse& res)
{
  uint32_t expo;
  uvc_error_t err = uvc_get_exposure_abs(devh_, &expo, UVC_GET_CUR);
  res.exposure = expo;
  return (err == UVC_SUCCESS);
}

bool CameraDriver::setUVCExposureCb(astra_camera::SetUVCExposureRequest& req, astra_camera::SetUVCExposureResponse& res)
{
  if (req.exposure == 0)
  {
    uvc_set_ae_mode(devh_, 2);
    return true;
  }
  uvc_set_ae_mode(devh_, 1); // mode 1: manual mode; 2: auto mode; 4: shutter priority mode; 8: aperture priority mode
  if (req.exposure > 330)
  {
    ROS_ERROR("Please set exposure lower than 330");
    return false;
  }
  uvc_error_t err = uvc_set_exposure_abs(devh_, req.exposure);
  return (err == UVC_SUCCESS);
}

bool CameraDriver::getUVCGainCb(astra_camera::GetUVCGainRequest& req, astra_camera::GetUVCGainResponse& res)
{
  uint16_t gain;
  uvc_error_t err = uvc_get_gain(devh_, &gain, UVC_GET_CUR);
  res.gain = gain;
  return (err == UVC_SUCCESS); 
}

bool CameraDriver::setUVCGainCb(astra_camera::SetUVCGainRequest& req, astra_camera::SetUVCGainResponse& res)
{
  uvc_error_t err = uvc_set_gain(devh_, req.gain);
  return (err == UVC_SUCCESS);
}

bool CameraDriver::getUVCWhiteBalanceCb(astra_camera::GetUVCWhiteBalanceRequest& req, astra_camera::GetUVCWhiteBalanceResponse& res)
{
  uint16_t white_balance;
  uvc_error_t err = uvc_get_white_balance_temperature(devh_, &white_balance, UVC_GET_CUR);
  res.white_balance = white_balance;
  return (err == UVC_SUCCESS);
}

bool CameraDriver::setUVCWhiteBalanceCb(astra_camera::SetUVCWhiteBalanceRequest& req, astra_camera::SetUVCWhiteBalanceResponse& res)
{
  if (req.white_balance == 0)
  {
    uvc_set_white_balance_temperature_auto(devh_, 1);
    return true;
  }
  uvc_set_white_balance_temperature_auto(devh_, 0); // 0: manual, 1: auto
  uvc_error_t err = uvc_set_white_balance_temperature(devh_, req.white_balance);
  return (err == UVC_SUCCESS);
}

bool CameraDriver::Start() {
  assert(state_ == kInitial);

  uvc_error_t err;

  err = uvc_init(&ctx_, NULL);

  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_init");
    return false;
  }

  state_ = kStopped;

  config_server_.setCallback(boost::bind(&CameraDriver::ReconfigureCallback, this, _1, _2));

  return state_ == kRunning;
}

void CameraDriver::Stop() {
  boost::recursive_mutex::scoped_lock(mutex_);

  assert(state_ != kInitial);

  if (state_ == kRunning)
    CloseCamera();

  assert(state_ == kStopped);

  uvc_exit(ctx_);
  ctx_ = NULL;

  state_ = kInitial;
}

void CameraDriver::ReconfigureCallback(UVCCameraConfig &new_config, uint32_t level) {
  boost::recursive_mutex::scoped_lock(mutex_);

  if ((level & kReconfigureClose) == kReconfigureClose) {
    if (state_ == kRunning)
      CloseCamera();
  }

  if (state_ == kStopped) {
    OpenCamera(new_config);
  }

  if (new_config.camera_info_url != config_.camera_info_url)
    cinfo_manager_.loadCameraInfo(new_config.camera_info_url);

  if (state_ == kRunning) {
#define PARAM_INT(name, fn, value) if (new_config.name != config_.name) { \
      int val = (value);                                                \
      if (uvc_set_##fn(devh_, val)) {                                   \
        ROS_WARN("Unable to set " #name " to %d", val);                 \
        new_config.name = config_.name;                                 \
      }                                                                 \
    }

    PARAM_INT(scanning_mode, scanning_mode, new_config.scanning_mode);
    PARAM_INT(auto_exposure, ae_mode, 1 << new_config.auto_exposure);
    PARAM_INT(auto_exposure_priority, ae_priority, new_config.auto_exposure_priority);
    PARAM_INT(exposure_absolute, exposure_abs, new_config.exposure_absolute * 10000);
    PARAM_INT(auto_focus, focus_auto, new_config.auto_focus ? 1 : 0);
    PARAM_INT(focus_absolute, focus_abs, new_config.focus_absolute);
#if libuvc_VERSION     > 00005 /* version > 0.0.5 */
    PARAM_INT(gain, gain, new_config.gain);
    PARAM_INT(iris_absolute, iris_abs, new_config.iris_absolute);
    PARAM_INT(brightness, brightness, new_config.brightness);
#endif
    

    if (new_config.pan_absolute != config_.pan_absolute || new_config.tilt_absolute != config_.tilt_absolute) {
      if (uvc_set_pantilt_abs(devh_, new_config.pan_absolute, new_config.tilt_absolute)) {
        ROS_WARN("Unable to set pantilt to %d, %d", new_config.pan_absolute, new_config.tilt_absolute);
        new_config.pan_absolute = config_.pan_absolute;
        new_config.tilt_absolute = config_.tilt_absolute;
      }
    }
    // TODO: roll_absolute
    // TODO: privacy
    // TODO: backlight_compensation
    // TODO: contrast
    // TODO: power_line_frequency
    // TODO: auto_hue
    // TODO: saturation
    // TODO: sharpness
    // TODO: gamma
    // TODO: auto_white_balance
    // TODO: white_balance_temperature
    // TODO: white_balance_BU
    // TODO: white_balance_RV
  }

  config_ = new_config;
}

void CameraDriver::ImageCallback(uvc_frame_t *frame) {
  ros::Time timestamp = ros::Time(frame->capture_time.tv_sec, frame->capture_time.tv_usec);
  if ( timestamp == ros::Time(0) ) {
    timestamp = ros::Time::now();
  }

  boost::recursive_mutex::scoped_lock(mutex_);

  assert(state_ == kRunning);
  assert(rgb_frame_);

  sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
  image->width = config_.width;
  image->height = config_.height;
  image->step = image->width * 3;
  image->data.resize(image->step * image->height);

  if (frame->frame_format == UVC_FRAME_FORMAT_BGR){
    image->encoding = "bgr8";
    memcpy(&(image->data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_RGB){
    image->encoding = "rgb8";
    memcpy(&(image->data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_UYVY) {
    image->encoding = "yuv422";
    memcpy(&(image->data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_YUYV) {
    // FIXME: uvc_any2bgr does not work on "yuyv" format, so use uvc_yuyv2bgr directly.
    uvc_error_t conv_ret = uvc_yuyv2bgr(frame, rgb_frame_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image->encoding = "bgr8";
    memcpy(&(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
#if libuvc_VERSION     > 00005 /* version > 0.0.5 */
  } else if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
    // Enable mjpeg support despite uvs_any2bgr shortcoming
    //  https://github.com/ros-drivers/libuvc_ros/commit/7508a09f
    uvc_error_t conv_ret = uvc_mjpeg2rgb(frame, rgb_frame_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image->encoding = "rgb8";
    memcpy(&(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
#endif
  } else {
    uvc_error_t conv_ret = uvc_any2bgr(frame, rgb_frame_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image->encoding = "bgr8";
    memcpy(&(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
  }

  astra_camera::GetDeviceType device_type_srv;
  astra_camera::GetCameraInfo camera_info_srv;

  if (device_type_init_ == false)
  {
    if (device_type_client.call(device_type_srv))
    {
      device_type_ = device_type_srv.response.device_type;
      if (strcmp(device_type_.c_str(), OB_STEREO_S) == 0)
      {
        device_type_no_ = OB_STEREO_S_NO;
      }
      else if (strcmp(device_type_.c_str(), OB_EMBEDDED_S) == 0)
      {
        device_type_no_ = OB_EMBEDDED_S_NO;
        uvc_flip_ = 1;
      }
      else if (strcmp(device_type_.c_str(), OB_ASTRA_PRO) == 0)
      {
        device_type_no_ = OB_ASTRA_PRO_NO;
      }
      else if (strcmp(device_type_.c_str(), OB_STEREO_S_U3) == 0)
      {
        device_type_no_ = OB_STEREO_S_U3_NO;
      }
      else
      {
        device_type_no_ = OB_ASTRA_NO;
      }
      device_type_init_ = true;
    }
  }

  if (camera_info_init_ == false)
  {
    if (camera_info_client.call(camera_info_srv))
    {
      camera_info_ = camera_info_srv.response.info;
      camera_info_init_ = true;
      camera_info_valid_ = true;
      if (std::isnan(camera_info_.K[0]) || std::isnan(camera_info_.K[2]) || std::isnan(camera_info_.K[4]) || std::isnan(camera_info_.K[5]))
      {
        camera_info_valid_ = false;
      }
    }
  }

  sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));
  if (device_type_init_ == true && astraWithUVC(device_type_no_))
  {
    // update cinfo
    if (camera_info_init_ == true && camera_info_valid_ == true)
    {
      cinfo->height = image->height;
      cinfo->width = image->width;
      cinfo->distortion_model = camera_info_.distortion_model;
      cinfo->D.resize(5, 0.0);
      cinfo->D[4] = 0.0000000001;
      // for (int i = 0; i < 5; i++)
      // {
      //   cinfo->D[i] = camera_info_.D[i];
      // }
      for (int i = 0; i < 9; i++)
      {
        cinfo->K[i] = camera_info_.K[i];
        cinfo->R[i] = camera_info_.R[i];
      }
      cinfo->K[0] = (1 - uvc_flip_)*(camera_info_.K[0]) + (uvc_flip_)*(-camera_info_.K[0]);
      cinfo->K[2] = (1 - uvc_flip_)*(camera_info_.K[2]) + (uvc_flip_)*(image->width - camera_info_.K[2]);
      for (int i = 0; i < 12; i++)
      {
        cinfo->P[i] = camera_info_.P[i];
      }
    }
    image->header.frame_id = ns_no_slash + "_rgb_optical_frame";
    cinfo->header.frame_id = ns_no_slash + "_rgb_optical_frame";
  }
  else
  {
    image->header.frame_id = config_.frame_id;
    cinfo->header.frame_id = config_.frame_id;
  }
  image->header.stamp = timestamp;
  cinfo->header.stamp = timestamp;

  cam_pub_.publish(image, cinfo);

  if (config_changed_)
  {
    config_server_.updateConfig(config_);
    config_changed_ = false;
  }
}

/* static */ void CameraDriver::ImageCallbackAdapter(uvc_frame_t *frame, void *ptr) {
  CameraDriver *driver = static_cast<CameraDriver*>(ptr);

  driver->ImageCallback(frame);
}

void CameraDriver::AutoControlsCallback(
  enum uvc_status_class status_class,
  int event,
  int selector,
  enum uvc_status_attribute status_attribute,
  void *data, size_t data_len) {
  boost::recursive_mutex::scoped_lock(mutex_);

  printf("Controls callback. class: %d, event: %d, selector: %d, attr: %d, data_len: %zu\n",
         status_class, event, selector, status_attribute, data_len);

  if (status_attribute == UVC_STATUS_ATTRIBUTE_VALUE_CHANGE) {
    switch (status_class) {
    case UVC_STATUS_CLASS_CONTROL_CAMERA: {
      switch (selector) {
      case UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
        uint8_t *data_char = (uint8_t*) data;
        uint32_t exposure_int = ((data_char[0]) | (data_char[1] << 8) |
                                 (data_char[2] << 16) | (data_char[3] << 24));
        config_.exposure_absolute = exposure_int * 0.0001;
        config_changed_ = true;
        break;
      }
      break;
    }
    case UVC_STATUS_CLASS_CONTROL_PROCESSING: {
      switch (selector) {
      case UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
        uint8_t *data_char = (uint8_t*) data;
        config_.white_balance_temperature = 
          data_char[0] | (data_char[1] << 8);
        config_changed_ = true;
        break;
      }
      break;
    }
    }

    // config_server_.updateConfig(config_);
  }
}

/* static */ void CameraDriver::AutoControlsCallbackAdapter(
  enum uvc_status_class status_class,
  int event,
  int selector,
  enum uvc_status_attribute status_attribute,
  void *data, size_t data_len,
  void *ptr) {
  CameraDriver *driver = static_cast<CameraDriver*>(ptr);

  driver->AutoControlsCallback(status_class, event, selector,
                               status_attribute, data, data_len);
}

enum uvc_frame_format CameraDriver::GetVideoMode(std::string vmode){
  if(vmode == "uncompressed") {
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  } else if (vmode == "compressed") {
    return UVC_COLOR_FORMAT_COMPRESSED;
  } else if (vmode == "yuyv") {
    return UVC_COLOR_FORMAT_YUYV;
  } else if (vmode == "uyvy") {
    return UVC_COLOR_FORMAT_UYVY;
  } else if (vmode == "rgb") {
    return UVC_COLOR_FORMAT_RGB;
  } else if (vmode == "bgr") {
    return UVC_COLOR_FORMAT_BGR;
  } else if (vmode == "mjpeg") {
    return UVC_COLOR_FORMAT_MJPEG;
  } else if (vmode == "gray8") {
    return UVC_COLOR_FORMAT_GRAY8;
  } else {
    ROS_ERROR_STREAM("Invalid Video Mode: " << vmode);
    ROS_WARN_STREAM("Continue using video mode: uncompressed");
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  }
};

void CameraDriver::OpenCamera(UVCCameraConfig &new_config) {
  assert(state_ == kStopped);

  int vendor_id = strtol(new_config.vendor.c_str(), NULL, 0);
  int product_id = strtol(new_config.product.c_str(), NULL, 0);

  ROS_INFO("Opening camera with vendor=0x%x, product=0x%x, serial=\"%s\", index=%d",
           vendor_id, product_id, new_config.serial.c_str(), new_config.index);

  uvc_device_t **devs;

  // Implement missing index select behavior
  // https://github.com/ros-drivers/libuvc_ros/commit/4f30e9a0
#if libuvc_VERSION     > 00005 /* version > 0.0.5 */
  uvc_error_t find_err = uvc_find_devices(
    ctx_, &devs,
    vendor_id,
    product_id,
    new_config.serial.empty() ? NULL : new_config.serial.c_str());

  if (find_err != UVC_SUCCESS) {
    uvc_perror(find_err, "uvc_find_device");
    return;
  }

  // select device by index
  dev_ = NULL;
  int dev_idx = 0;
  while (devs[dev_idx] != NULL) {
    if(dev_idx == new_config.index) {
      dev_ = devs[dev_idx];
    }
    else {
      uvc_unref_device(devs[dev_idx]);
    }

    dev_idx++;
  }

  if(dev_ == NULL) {
    ROS_ERROR("Unable to find device at index %d", new_config.index);
    return;
  }
#else
  uvc_error_t find_err = uvc_find_device(
    ctx_, &dev_,
    vendor_id,
    product_id,
    new_config.serial.empty() ? NULL : new_config.serial.c_str());

  if (find_err != UVC_SUCCESS) {
    uvc_perror(find_err, "uvc_find_device");
    return;
  }

#endif
  uvc_error_t open_err = uvc_open(dev_, &devh_);

  if (open_err != UVC_SUCCESS) {
    switch (open_err) {
    case UVC_ERROR_ACCESS:
#ifdef __linux__
      ROS_ERROR("Permission denied opening /dev/bus/usb/%03d/%03d",
                uvc_get_bus_number(dev_), uvc_get_device_address(dev_));
#else
      ROS_ERROR("Permission denied opening device %d on bus %d",
                uvc_get_device_address(dev_), uvc_get_bus_number(dev_));
#endif
      break;
    default:
#ifdef __linux__
      ROS_ERROR("Can't open /dev/bus/usb/%03d/%03d: %s (%d)",
                uvc_get_bus_number(dev_), uvc_get_device_address(dev_),
                uvc_strerror(open_err), open_err);
#else
      ROS_ERROR("Can't open device %d on bus %d: %s (%d)",
                uvc_get_device_address(dev_), uvc_get_bus_number(dev_),
                uvc_strerror(open_err), open_err);
#endif
      break;
    }

    uvc_unref_device(dev_);
    return;
  }

  uvc_set_status_callback(devh_, &CameraDriver::AutoControlsCallbackAdapter, this);

  uvc_stream_ctrl_t ctrl;
  uvc_error_t mode_err = uvc_get_stream_ctrl_format_size(
    devh_, &ctrl,
    GetVideoMode(new_config.video_mode),
    new_config.width, new_config.height,
    new_config.frame_rate);

  if (mode_err != UVC_SUCCESS) {
    uvc_perror(mode_err, "uvc_get_stream_ctrl_format_size");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    ROS_ERROR("check video_mode/width/height/frame_rate are available");
    uvc_print_diag(devh_, NULL);
    return;
  }

  uvc_error_t stream_err = uvc_start_streaming(devh_, &ctrl, &CameraDriver::ImageCallbackAdapter, this, 0);

  if (stream_err != UVC_SUCCESS) {
    uvc_perror(stream_err, "uvc_start_streaming");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    return;
  }

  if (rgb_frame_)
    uvc_free_frame(rgb_frame_);

  rgb_frame_ = uvc_allocate_frame(new_config.width * new_config.height * 3);
  assert(rgb_frame_);

  state_ = kRunning;
}

void CameraDriver::CloseCamera() {
  assert(state_ == kRunning);

  uvc_close(devh_);
  devh_ = NULL;

  uvc_unref_device(dev_);
  dev_ = NULL;

  state_ = kStopped;
}

};
