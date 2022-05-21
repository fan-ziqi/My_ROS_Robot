#include "ros2_astra_camera/camera_driver.h"

#include <libuvc/libuvc.h>

#include <cmath>
#include <image_transport/camera_publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#define libuvc_VERSION                                         \
  (libuvc_VERSION_MAJOR * 10000 + libuvc_VERSION_MINOR * 100 + \
   libuvc_VERSION_PATCH)

namespace ros2_astra_camera {
CameraDriver::CameraDriver(const rclcpp::NodeOptions &options)
    : rclcpp::Node("astra_uvc_camera", options),
      state_(kInitial),
      ctx_(NULL),
      dev_(NULL),
      devh_(NULL),
      rgb_frame_(NULL),
      config_changed_(false),
      param_init_(false) {
  initParameters();

  initServices();

  dyn_params_handler_ = add_on_set_parameters_callback(std::bind(
      &CameraDriver::dynamicParametersCallback, this, std::placeholders::_1));

  cam_pub_ =
      image_transport::create_camera_publisher(this, "camera/color/image_raw");

  client_node_ =
      rclcpp::Node::make_shared(std::string(this->get_name()) + "_client_node");
  device_type_client_ =
      create_client<astra_msgs::srv::GetDeviceType>("get_device_type");
  camera_info_client_ =
      create_client<astra_msgs::srv::GetCameraInfo>("get_camera_info");

  device_type_init_ = false;
  camera_info_init_ = false;
  uvc_flip_ = 0;
  device_type_no_ = OB_ASTRA_NO;
  // int slash_end;
  // for (slash_end = 0; slash_end < ns.length(); slash_end++) {
  //   if (ns[slash_end] != '/') {
  //     break;
  //   }
  // }
  // ns_no_slash = ns.substr(slash_end);
  ns_no_slash = "camera";
  camera_info_valid_ = false;

  if (!this->Start()) {
    throw std::runtime_error("camera driver start failed.");
  }
}

CameraDriver::~CameraDriver() {
  this->Stop();

  if (rgb_frame_) uvc_free_frame(rgb_frame_);

  if (ctx_) uvc_exit(ctx_);  // Destroys dev_, devh_, etc.
}

void CameraDriver::initParameters() {
  config_.vendor = this->declare_parameter<std::string>("vendor", "0x2bc5");
  config_.product = this->declare_parameter<std::string>("product", "0x0502");
  config_.serial = this->declare_parameter<std::string>("serial", "");
  config_.index = this->declare_parameter<int>("index", 0);
  config_.width = this->declare_parameter<int>("width", 640);
  config_.height = this->declare_parameter<int>("height", 480);
  config_.video_mode =
      this->declare_parameter<std::string>("video_mode", "uncompressed");
  config_.frame_rate = this->declare_parameter<double>("frame_rate", 30.0);
  config_.timestamp_method =
      this->declare_parameter<std::string>("timestamp_method", "start");
  config_.frame_id = this->declare_parameter<std::string>("frame_id", "camera");
  config_.camera_info_url =
      this->declare_parameter<std::string>("camera_info_url", "");
  config_.scanning_mode = this->declare_parameter<int>("scanning_mode", 0);
  config_.auto_exposure = this->declare_parameter<int>("auto_exposure", 3);
  config_.auto_exposure_priority =
      this->declare_parameter<int>("auto_exposure_priority", 0);
  config_.exposure_absolute =
      this->declare_parameter<double>("exposure_absolute", 0.0);
  config_.iris_absolute = this->declare_parameter<double>("iris_absolute", 0.0);
  config_.auto_focus = this->declare_parameter<bool>("auto_focus", true);
  config_.focus_absolute = this->declare_parameter<int>("focus_absolute", 0);
  config_.pan_absolute = this->declare_parameter<int>("pan_absolute", 0);
  config_.tilt_absolute = this->declare_parameter<int>("tilt_absolute", 0);
  config_.roll_absolute = this->declare_parameter<int>("roll_absolute", 0);
  config_.privacy = this->declare_parameter<bool>("privacy", false);
  config_.backlight_compensation =
      this->declare_parameter<int>("backlight_compensation", 0);
  config_.brightness = this->declare_parameter<int>("brightness", 0);
  config_.contrast = this->declare_parameter<int>("contrast", 0);
  config_.gain = this->declare_parameter<int>("gain", 0);
  config_.power_line_frequency =
      this->declare_parameter<int>("power_line_frequency", 0);
  config_.auto_hue = this->declare_parameter<bool>("auto_hue", false);
  config_.hue = this->declare_parameter<double>("hue", 0.0);
  config_.saturation = this->declare_parameter<int>("saturation", 0);
  config_.sharpness = this->declare_parameter<int>("sharpness", 0);
  config_.gamma = this->declare_parameter<double>("gamma", 1.0);
  config_.auto_white_balance =
      this->declare_parameter<bool>("auto_white_balance", false);
  config_.white_balance_temperature =
      this->declare_parameter<int>("white_balance_temperature", 0);
  config_.white_balance_BU =
      this->declare_parameter<int>("white_balance_BU", 0);
  config_.white_balance_RV =
      this->declare_parameter<int>("white_balance_RV", 0);
}
void CameraDriver::initServices() {
  get_uvc_exposure_srv_ = create_service<astra_msgs::srv::GetUVCExposure>(
      "get_uvc_exposure",
      std::bind(&CameraDriver::getUVCExposureCallback, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));
  set_uvc_exposure_srv_ = create_service<astra_msgs::srv::SetUVCExposure>(
      "set_uvc_exposure",
      std::bind(&CameraDriver::setUVCExposureCallback, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));
  get_uvc_gain_srv_ = create_service<astra_msgs::srv::GetUVCGain>(
      "get_uvc_gain",
      std::bind(&CameraDriver::getUVCGainCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  set_uvc_gain_srv_ = create_service<astra_msgs::srv::SetUVCGain>(
      "set_uvc_gain",
      std::bind(&CameraDriver::setUVCGainCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  get_uvc_white_balance_srv_ =
      create_service<astra_msgs::srv::GetUVCWhiteBalance>(
          "get_uvc_white_balance",
          std::bind(&CameraDriver::getUVCWhiteBalanceCallback, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3));
  set_uvc_white_balance_srv_ =
      create_service<astra_msgs::srv::SetUVCWhiteBalance>(
          "set_uvc_white_balance",
          std::bind(&CameraDriver::setUVCWhiteBalanceCallback, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3));
  set_uvc_auto_exposure_srv_ = create_service<astra_msgs::srv::SetAutoExposure>(
      "set_uvc_auto_exposure",
      std::bind(&CameraDriver::setAutoExposureCallback, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));
  set_uvc_auto_white_balance_srv_ =
      create_service<astra_msgs::srv::SetAutoWhiteBalance>(
          "set_uvc_auto_white_balance",
          std::bind(&CameraDriver::setAutoWhiteBalanceCallback, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3));
}

rcl_interfaces::msg::SetParametersResult
CameraDriver::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  old_config_ = config_;

  // if ((level & kReconfigureClose) == kReconfigureClose) {
  // if (state_ == kRunning) CloseCamera();
  // }

  for (const auto &config : parameters) {
    if (config.get_name() == "camera_info_url" &&
        config.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      if (config_.camera_info_url != config.get_value<std::string>()) {
        config_.camera_info_url = config.get_value<std::string>();
        color_info_manager_->loadCameraInfo(config_.camera_info_url);
      }
    } else if (state_ == kRunning) {
      if (config.get_name() == "scanning_mode" &&
          config.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        int val = config.get_value<int>();

        if (uvc_set_scanning_mode(devh_, val)) {
          RCLCPP_WARN(get_logger(), "Unable to set \"%s\" to %d",
                      config.get_name().c_str(), val);
        } else {
          config_.scanning_mode = val;
        }
      } else if (config.get_name() == "auto_exposure" &&
                 config.get_type() ==
                     rclcpp::ParameterType::PARAMETER_INTEGER) {
        int val = config.get_value<int>();
        if (uvc_set_ae_mode(devh_, val)) {
          RCLCPP_WARN(get_logger(), "Unable to set \"%s\" to %d",
                      config.get_name().c_str(), val);
        } else {
          config_.auto_exposure = val;
        }
      } else if (config.get_name() == "auto_exposure_priority" &&
                 config.get_type() ==
                     rclcpp::ParameterType::PARAMETER_INTEGER) {
        int val = config.get_value<int>();
        if (uvc_set_ae_priority(devh_, val)) {
          RCLCPP_WARN(get_logger(), "Unable to set \"%s\" to %d",
                      config.get_name().c_str(), val);
        } else {
          config_.auto_exposure_priority = val;
        }
      } else if (config.get_name() == "exposure_absolute" &&
                 config.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        double val = config.get_value<double>();
        if (uvc_set_exposure_abs(devh_, val)) {
          RCLCPP_WARN(get_logger(), "Unable to set \"%s\" to %f",
                      config.get_name().c_str(), val);
        } else {
          config_.exposure_absolute = val;
        }
      } else if (config.get_name() == "auto_focus" &&
                 config.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        bool val = config.get_value<bool>();
        if (uvc_set_focus_auto(devh_, val)) {
          RCLCPP_WARN(get_logger(), "Unable to set \"%s\" to %s",
                      config.get_name().c_str(), val ? "true" : "false");
        } else {
          config_.auto_focus = val;
        }
      } else if (config.get_name() == "focus_absolute" &&
                 config.get_type() ==
                     rclcpp::ParameterType::PARAMETER_INTEGER) {
        int val = config.get_value<int>();
        if (uvc_set_focus_abs(devh_, val)) {
          RCLCPP_WARN(get_logger(), "Unable to set \"%s\" to %d",
                      config.get_name().c_str(), val);
        } else {
          config_.focus_absolute = val;
        }
      }
#if libuvc_VERSION > 00005 /* version > 0.0.5 */
      else if (config.get_name() == "gain" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        int val = config.get_value<int>();
        if (uvc_set_gain(devh_, val)) {
          RCLCPP_WARN(get_logger(), "Unable to set \"%s\" to %d",
                      config.get_name().c_str(), val);
        } else {
          config_.gain = val;
        }
      } else if (config.get_name() == "iris_absolute" &&
                 config.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        double val = config.get_value<double>();
        if (uvc_set_iris_abs(devh_, val)) {
          RCLCPP_WARN(get_logger(), "Unable to set \"%s\" to %f",
                      config.get_name().c_str(), val);
        } else {
          config_.iris_absolute = val;
        }
      } else if (config.get_name() == "brightness" &&
                 config.get_type() ==
                     rclcpp::ParameterType::PARAMETER_INTEGER) {
        int val = config.get_value<int>();
        if (uvc_set_brightness(devh_, val)) {
          RCLCPP_WARN(get_logger(), "Unable to set \"%s\" to %d",
                      config.get_name().c_str(), val);
        } else {
          config_.brightness = val;
        }
      }
#endif
      else if (config.get_name() == "pan_absolute" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        config_.pan_absolute = config.get_value<int>();
      } else if (config.get_name() == "tilt_absolute" &&
                 config.get_type() ==
                     rclcpp::ParameterType::PARAMETER_INTEGER) {
        config_.tilt_absolute = config.get_value<int>();
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
  }

  if (old_config_.pan_absolute != config_.pan_absolute ||
      old_config_.tilt_absolute != config_.tilt_absolute) {
    if (uvc_set_pantilt_abs(devh_, config_.pan_absolute,
                            config_.tilt_absolute)) {
      RCLCPP_WARN(get_logger(), "Unable to set pantilt to %d, %d",
                  config_.pan_absolute, config_.tilt_absolute);
      config_.pan_absolute = old_config_.pan_absolute;
      config_.tilt_absolute = old_config_.tilt_absolute;
    }
  }

  if (state_ == kStopped) {
    OpenCamera(config_);
  }

  return result;
}

void CameraDriver::getUVCExposureCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::GetUVCExposure::Request> /*req*/,
    std::shared_ptr<astra_msgs::srv::GetUVCExposure::Response> res) {
  uint32_t expo;
  uvc_get_exposure_abs(devh_, &expo, UVC_GET_CUR);
  res->exposure = expo;
}

void CameraDriver::setUVCExposureCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetUVCExposure::Request> req,
    std::shared_ptr<astra_msgs::srv::SetUVCExposure::Response> res) {
  if (req->exposure == 0) {
    uvc_set_ae_mode(devh_, 2);
    res->success = true;
    return;
  }
  uvc_set_ae_mode(devh_, 1);  // mode 1: manual mode; 2: auto mode; 4: shutter
                              // priority mode; 8: aperture priority mode
  if (req->exposure > 330) {
    RCLCPP_ERROR(get_logger(), "Please set exposure lower than 330");
    res->success = false;
    return;
  }

  uvc_error_t err = uvc_set_exposure_abs(devh_, req->exposure);
  res->success = err == UVC_SUCCESS;
}

void CameraDriver::getUVCGainCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::GetUVCGain::Request> /*req*/,
    std::shared_ptr<astra_msgs::srv::GetUVCGain::Response> res) {
  uint16_t gain;
  uvc_get_gain(devh_, &gain, UVC_GET_CUR);
  res->gain = gain;
}

void CameraDriver::setUVCGainCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetUVCGain::Request> req,
    std::shared_ptr<astra_msgs::srv::SetUVCGain::Response> res) {
  uvc_error_t err = uvc_set_gain(devh_, req->gain);
  res->success = err == UVC_SUCCESS;
}

void CameraDriver::getUVCWhiteBalanceCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::GetUVCWhiteBalance::Request> /*req*/,
    std::shared_ptr<astra_msgs::srv::GetUVCWhiteBalance::Response> res) {
  uint16_t white_balance;
  uvc_get_white_balance_temperature(devh_, &white_balance, UVC_GET_CUR);
  res->white_balance = white_balance;
}

void CameraDriver::setUVCWhiteBalanceCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetUVCWhiteBalance::Request> req,
    std::shared_ptr<astra_msgs::srv::SetUVCWhiteBalance::Response> res) {
  if (req->white_balance == 0) {
    uvc_set_white_balance_temperature_auto(devh_, 1);
    res->success = true;
    return;
  }
  uvc_set_white_balance_temperature_auto(devh_, 0);  // 0: manual, 1: auto
  uvc_error_t err =
      uvc_set_white_balance_temperature(devh_, req->white_balance);
  res->success = err == UVC_SUCCESS;
}

void CameraDriver::setAutoExposureCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetAutoExposure::Request> req,
    std::shared_ptr<astra_msgs::srv::SetAutoExposure::Response> /*res*/) {
  if (req->enable) {
    uvc_set_ae_mode(devh_, 2);
  } else {
    uvc_set_ae_mode(devh_, 1);
  }
}

void CameraDriver::setAutoWhiteBalanceCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetAutoWhiteBalance::Request> req,
    std::shared_ptr<astra_msgs::srv::SetAutoWhiteBalance::Response> /*res*/) {
  if (req->enable) {
    uvc_set_white_balance_temperature_auto(devh_, 1);
  } else {
    uvc_set_white_balance_temperature_auto(devh_, 0);
  }
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

  dynamicParametersCallback({});

  return state_ == kRunning;
}

void CameraDriver::Stop() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  assert(state_ != kInitial);

  if (state_ == kRunning) CloseCamera();

  assert(state_ == kStopped);

  uvc_exit(ctx_);
  ctx_ = NULL;

  state_ = kInitial;
}

void CameraDriver::ImageCallback(uvc_frame_t *frame) {
  rclcpp::Time timestamp =
      rclcpp::Time(frame->capture_time.tv_sec, frame->capture_time.tv_usec);
  if (timestamp == rclcpp::Time(0)) {
    timestamp = this->now();
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);

  assert(state_ == kRunning);
  assert(rgb_frame_);

  sensor_msgs::msg::Image::SharedPtr image(new sensor_msgs::msg::Image());
  image->width = config_.width;
  image->height = config_.height;
  image->step = image->width * 3;
  image->data.resize(image->step * image->height);

  if (frame->frame_format == UVC_FRAME_FORMAT_BGR) {
    image->encoding = "bgr8";
    memcpy(&(image->data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_RGB) {
    image->encoding = "rgb8";
    memcpy(&(image->data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_UYVY) {
    image->encoding = "yuv422";
    memcpy(&(image->data[0]), frame->data, frame->data_bytes);
  } else if (frame->frame_format == UVC_FRAME_FORMAT_YUYV) {
    // FIXME: uvc_any2bgr does not work on "yuyv" format, so use uvc_yuyv2bgr
    // directly.
    uvc_error_t conv_ret = uvc_yuyv2bgr(frame, rgb_frame_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image->encoding = "bgr8";
    memcpy(&(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
#if libuvc_VERSION > 00005 /* version > 0.0.5 */
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

  if (device_type_init_ == false) {
    if (device_type_client_->wait_for_service(std::chrono::seconds(5))) {
      auto request =
          std::make_shared<astra_msgs::srv::GetDeviceType::Request>();
      auto future_result = device_type_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(client_node_, future_result,
                                             std::chrono::seconds(5)) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future_result.get();
        if (response.get()) {
          device_type_ = response->device_type;

          if (strcmp(device_type_.c_str(), OB_STEREO_S) == 0) {
            device_type_no_ = OB_STEREO_S_NO;
          } else if (strcmp(device_type_.c_str(), OB_EMBEDDED_S) == 0) {
            device_type_no_ = OB_EMBEDDED_S_NO;
            uvc_flip_ = 1;
          } else if (strcmp(device_type_.c_str(), OB_ASTRA_PRO) == 0) {
            device_type_no_ = OB_ASTRA_PRO_NO;
          } else if (strcmp(device_type_.c_str(), OB_ASTRA_PRO_PLUS) == 0) {
            device_type_no_ = OB_ASTRA_PRO_PLUS_NO;
          } else if (strcmp(device_type_.c_str(), OB_STEREO_S_U3) == 0) {
            device_type_no_ = OB_STEREO_S_U3_NO;
          } else if (strcmp(device_type_.c_str(), OB_DABAI) == 0) {
            device_type_no_ = OB_DABAI_NO;
          } else if (strcmp(device_type_.c_str(), OB_ASTRA_PLUS) == 0) {
            device_type_no_ = OB_ASTRA_PLUS_NO;
          } else {
            device_type_no_ = OB_ASTRA_NO;
          }
          device_type_init_ = true;
          color_info_manager_ =
              std::make_shared<camera_info_manager::CameraInfoManager>(
                  this, "color_" + response->string_id,
                  config_.camera_info_url);
        }
      }
    }
  }

  if (camera_info_init_ == false) {
    if (camera_info_client_->wait_for_service(std::chrono::seconds(5))) {
      auto request =
          std::make_shared<astra_msgs::srv::GetCameraInfo::Request>();
      auto future_result = camera_info_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(client_node_, future_result,
                                             std::chrono::seconds(5)) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future_result.get();
        if (response.get()) {
          camera_info_ = response->info;
          camera_info_init_ = true;
          camera_info_valid_ = true;
          if (std::isnan(camera_info_.k[0]) || std::isnan(camera_info_.k[2]) ||
              std::isnan(camera_info_.k[4]) || std::isnan(camera_info_.k[5])) {
            camera_info_valid_ = false;
          }
        }
      }
    }
  }

  if (!camera_info_init_ || !device_type_init_) {
    return;
  }

  sensor_msgs::msg::CameraInfo::SharedPtr cinfo(
      new sensor_msgs::msg::CameraInfo(color_info_manager_->getCameraInfo()));
  if (device_type_init_ == true && astraWithUVC(device_type_no_)) {
    // update cinfo
    if (camera_info_init_ == true && camera_info_valid_ == true) {
      cinfo->height = image->height;
      cinfo->width = image->width;
      cinfo->distortion_model = camera_info_.distortion_model;
      cinfo->d.resize(5, 0.0);
      cinfo->d[4] = 0.0000000001;
      // for (int i = 0; i < 5; i++)
      // {
      //   cinfo->d[i] = camera_info_.d[i];
      // }
      for (int i = 0; i < 9; i++) {
        cinfo->k[i] = camera_info_.k[i];
        cinfo->r[i] = camera_info_.r[i];
      }
      cinfo->k[0] = (1 - uvc_flip_) * (camera_info_.k[0]) +
                    (uvc_flip_) * (-camera_info_.k[0]);
      cinfo->k[2] = (1 - uvc_flip_) * (camera_info_.k[2]) +
                    (uvc_flip_) * (image->width - camera_info_.k[2]);
      for (int i = 0; i < 12; i++) {
        cinfo->p[i] = camera_info_.p[i];
      }
    }
    image->header.frame_id = ns_no_slash + "_rgb_optical_frame";
    cinfo->header.frame_id = ns_no_slash + "_rgb_optical_frame";
  } else {
    image->header.frame_id = config_.frame_id;
    cinfo->header.frame_id = config_.frame_id;
  }
  image->header.stamp = timestamp;
  cinfo->header.stamp = timestamp;

  cam_pub_.publish(image, cinfo);

  if (config_changed_) {
    config_changed_ = false;
  }
}

/* static */ void CameraDriver::ImageCallbackAdapter(uvc_frame_t *frame,
                                                     void *ptr) {
  CameraDriver *driver = static_cast<CameraDriver *>(ptr);

  driver->ImageCallback(frame);
}

void CameraDriver::AutoControlsCallback(
    enum uvc_status_class status_class, int event, int selector,
    enum uvc_status_attribute status_attribute, void *data, size_t data_len) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  printf(
      "Controls callback. class: %d, event: %d, selector: %d, attr: %d, "
      "data_len: %zu\n",
      status_class, event, selector, status_attribute, data_len);

  if (status_attribute == UVC_STATUS_ATTRIBUTE_VALUE_CHANGE) {
    switch (status_class) {
      case UVC_STATUS_CLASS_CONTROL_CAMERA: {
        switch (selector) {
          case UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
            uint8_t *data_char = (uint8_t *)data;
            uint32_t exposure_int =
                ((data_char[0]) | (data_char[1] << 8) | (data_char[2] << 16) |
                 (data_char[3] << 24));
            config_.exposure_absolute = exposure_int * 0.0001;
            config_changed_ = true;
            break;
        }
        break;
      }
      case UVC_STATUS_CLASS_CONTROL_PROCESSING: {
        switch (selector) {
          case UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
            uint8_t *data_char = (uint8_t *)data;
            config_.white_balance_temperature =
                data_char[0] | (data_char[1] << 8);
            config_changed_ = true;
            break;
        }
        break;
      }
      default: {
        // TODO: handle UVC_STATUS_CLASS_CONTROL
      }
    }
  }
}

/* static */ void CameraDriver::AutoControlsCallbackAdapter(
    enum uvc_status_class status_class, int event, int selector,
    enum uvc_status_attribute status_attribute, void *data, size_t data_len,
    void *ptr) {
  CameraDriver *driver = static_cast<CameraDriver *>(ptr);

  driver->AutoControlsCallback(status_class, event, selector, status_attribute,
                               data, data_len);
}

enum uvc_frame_format CameraDriver::GetVideoMode(std::string vmode) {
  if (vmode == "uncompressed") {
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
    RCLCPP_ERROR_STREAM(get_logger(), "Invalid Video Mode: " << vmode);
    RCLCPP_WARN_STREAM(get_logger(), "Continue using video mode: uncompressed");
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  }
}

void CameraDriver::OpenCamera(UVCCameraConfig &new_config) {
  assert(state_ == kStopped);

  int vendor_id = strtol(new_config.vendor.c_str(), NULL, 0);
  int product_id = strtol(new_config.product.c_str(), NULL, 0);
  //  ROS_INFO_STREAM("------------>Opening camera with vendor=" <<
  //  new_config.serial.c_str()); RCLCPP_INFO(get_logger(),"OpenCamera
  //  OpenCamera OpenCamera ---- %s",new_config.serial.c_str());
  RCLCPP_INFO(get_logger(),
              "Opening camera with vendor=0x%x, product=0x%x, serial=\"%s\", "
              "index=%d",
              vendor_id, product_id, new_config.serial.c_str(),
              new_config.index);

  // Implement missing index select behavior
  // https://github.com/ros-drivers/libuvc_ros/commit/4f30e9a0
#if libuvc_VERSION > 00005 /* version > 0.0.5 */
  uvc_device_t **devs;

  uvc_error_t find_err = uvc_find_devices(
      ctx_, &devs, vendor_id, product_id,
      new_config.serial.empty() ? NULL : new_config.serial.c_str());

  if (find_err != UVC_SUCCESS) {
    uvc_perror(find_err, "uvc_find_device");
    return;
  }

  // select device by index
  dev_ = NULL;
  int dev_idx = 0;
  while (devs[dev_idx] != NULL) {
    if (dev_idx == new_config.index) {
      dev_ = devs[dev_idx];
    } else {
      uvc_unref_device(devs[dev_idx]);
    }

    dev_idx++;
  }

  if (dev_ == NULL) {
    RCLCPP_ERROR(get_logger(), "Unable to find device at index %d",
                 new_config.index);
    return;
  }
#else
  uvc_error_t find_err = uvc_find_device(
      ctx_, &dev_, vendor_id, product_id,
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
        RCLCPP_ERROR(get_logger(),
                     "Permission denied opening /dev/bus/usb/%03d/%03d",
                     uvc_get_bus_number(dev_), uvc_get_device_address(dev_));
#else
        RCLCPP_ERROR(get_logger(),
                     "Permission denied opening device %d on bus %d",
                     uvc_get_device_address(dev_), uvc_get_bus_number(dev_));
#endif
        break;
      default:
#ifdef __linux__
        RCLCPP_ERROR(get_logger(), "Can't open /dev/bus/usb/%03d/%03d: %s (%d)",
                     uvc_get_bus_number(dev_), uvc_get_device_address(dev_),
                     uvc_strerror(open_err), open_err);
#else
        RCLCPP_ERROR(get_logger(), "Can't open device %d on bus %d: %s (%d)",
                     uvc_get_device_address(dev_), uvc_get_bus_number(dev_),
                     uvc_strerror(open_err), open_err);
#endif
        break;
    }

    uvc_unref_device(dev_);
    return;
  }

  uvc_set_status_callback(devh_, &CameraDriver::AutoControlsCallbackAdapter,
                          this);

  uvc_stream_ctrl_t ctrl;
  uvc_error_t mode_err = uvc_get_stream_ctrl_format_size(
      devh_, &ctrl, GetVideoMode(new_config.video_mode), new_config.width,
      new_config.height, new_config.frame_rate);

  RCLCPP_INFO(get_logger(), "uvc mode: %dx%d@%f %s", new_config.width,
              new_config.height, new_config.frame_rate,
              new_config.video_mode.c_str());

  if (mode_err != UVC_SUCCESS) {
    uvc_perror(mode_err, "uvc_get_stream_ctrl_format_size");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    RCLCPP_ERROR(get_logger(),
                 "check video_mode/width/height/frame_rate are available");
    uvc_print_diag(devh_, NULL);
    return;
  }

  uvc_error_t stream_err = uvc_start_streaming(
      devh_, &ctrl, &CameraDriver::ImageCallbackAdapter, this, 0);

  if (stream_err != UVC_SUCCESS) {
    uvc_perror(stream_err, "uvc_start_streaming");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    return;
  }

  if (rgb_frame_) uvc_free_frame(rgb_frame_);

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

}  // namespace ros2_astra_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_astra_camera::CameraDriver)
