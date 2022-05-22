#include "ros2_astra_camera/astra_driver.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/shm.h>
#include <unistd.h>

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "ros2_astra_camera/astra_wrapper/astra_device_type.h"
#include "ros2_astra_camera/astra_wrapper/astra_exception.h"
#include "ros2_astra_camera/astra_wrapper/astra_version.h"

#define MULTI_ASTRA 1
namespace ros2_astra_camera {
AstraDriver::AstraDriver(const rclcpp::NodeOptions& options)
    : rclcpp::Node("astra_camera", options),
      device_manager_(AstraDeviceManager::getSingelton()),
      config_init_(false),
      data_skip_ir_counter_(0),
      data_skip_color_counter_(0),
      data_skip_depth_counter_(0),
      ir_subscribers_(false),
      color_subscribers_(false),
      depth_subscribers_(false),
      depth_raw_subscribers_(false),
      depth_time_offset_(rclcpp::Duration::from_seconds(0.0)),
      ir_time_offset_(rclcpp::Duration::from_seconds(0.0)),
      color_time_offset_(rclcpp::Duration::from_seconds(0.0)) {
  genVideoModeTableMap();

  initParameters();

#if MULTI_ASTRA
  int bootOrder, devnums;

  bootOrder = this->declare_parameter<int>("bootorder", 0);
  devnums = this->declare_parameter<int>("devnums", 1);

  if (devnums > 1) {
    int shmid;
    char* shm = NULL;
    char* tmp;

    if (bootOrder == 1) {
      if ((shmid = shmget((key_t)0401, 1, 0666 | IPC_CREAT)) == -1) {
        RCLCPP_ERROR(get_logger(), "Create Share Memory Error:%s",
                     strerror(errno));
      }
      shm = (char*)shmat(shmid, 0, 0);
      *shm = 1;
      initDevice();
      RCLCPP_INFO(get_logger(),
                  "*********** device_id %s already open "
                  "device************************ ",
                  device_id_.c_str());
      *shm = 2;
    } else {
      if ((shmid = shmget((key_t)0401, 1, 0666 | IPC_CREAT)) == -1) {
        RCLCPP_ERROR(get_logger(), "Create Share Memory Error:%s",
                     strerror(errno));
      }
      shm = (char*)shmat(shmid, 0, 0);
      while (*shm != bootOrder) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      initDevice();
      RCLCPP_INFO(get_logger(),
                  "*********** device_id %s already open "
                  "device************************ ",
                  device_id_.c_str());
      *shm = (bootOrder + 1);
    }
    if (bootOrder == devnums) {
      if (shmdt(shm) == -1) {
        RCLCPP_ERROR(get_logger(), "shmdt failed\n");
      }
      if (shmctl(shmid, IPC_RMID, 0) == -1) {
        RCLCPP_ERROR(get_logger(), "shmctl(IPC_RMID) failed\n");
      }
    } else {
      if (shmdt(shm) == -1) {
        RCLCPP_ERROR(get_logger(), "shmdt failed\n");
      }
    }
  } else {
    initDevice();
  }
#else
  initDevice();

#endif

  // Using initial parameters to do initialization
  dynamicParametersCallback({});

  while (!config_init_ && rclcpp::ok()) {
    RCLCPP_DEBUG(get_logger(),
                 "Waiting for dynamic reconfigure configuration.");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!rclcpp::ok()) {
    exit(0);
  }

  RCLCPP_DEBUG(get_logger(), "Dynamic reconfigure configuration received.");

  dyn_params_handler_ = add_on_set_parameters_callback(std::bind(
      &AstraDriver::dynamicParametersCallback, this, std::placeholders::_1));

  initServices();

  advertiseROSTopics();
}

AstraDriver::~AstraDriver() { device_->stopAllStreams(); }

void AstraDriver::advertiseROSTopics() {
  std::lock_guard<std::recursive_mutex> lock(connect_mutex_);

  // Asus Xtion PRO does not have an RGB camera
  // RCLCPP_WARN(get_logger(), "has color sensor is %d",
  // device_->hasColorSensor()); if (device_->hasColorSensor()) {
  //   pub_color_ = image_transport::create_camera_publisher(this,
  //   "color/image"); imageConnectCb();
  // }

  RCLCPP_WARN(get_logger(), "has IR sensor is %d", device_->hasIRSensor());
  if (device_->hasIRSensor()) {
    pub_ir_ = image_transport::create_camera_publisher(this, "camera/ir/image");
    imageConnectCb();
  }

  RCLCPP_WARN(get_logger(), "has depth sensor is %d",
              device_->hasDepthSensor());
  if (device_->hasDepthSensor()) {
    pub_depth_raw_ = image_transport::create_camera_publisher(
        this, "camera/depth/image_raw");
    pub_depth_ =
        image_transport::create_camera_publisher(this, "camera/depth/image");

    pub_projector_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/projector/camera_info", rclcpp::SystemDefaultsQoS());
    depthConnectCb();
  }

  std::string string_id = device_->getStringID();
  std::string color_name, ir_name;

  color_name = "rgb_" + string_id;
  ir_name = "depth_" + string_id;

  // Load the saved calibrations, if they exist
  color_info_manager_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(this, color_name,
                                                               color_info_url_);
  ir_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, ir_name, ir_info_url_);
}

void AstraDriver::getCameraInfoCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::GetCameraInfo::Request> /*req*/,
    std::shared_ptr<astra_msgs::srv::GetCameraInfo::Response> res) {
  res->info = convertAstraCameraInfo(device_->getCameraParams(), this->now());
}

void AstraDriver::getSerialCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::GetSerial::Request> /*req*/,
    std::shared_ptr<astra_msgs::srv::GetSerial::Response> res) {
  res->serial = device_manager_->getSerial(device_->getUri());
}

void AstraDriver::getDeviceTypeCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::GetDeviceType::Request> /*req*/,
    std::shared_ptr<astra_msgs::srv::GetDeviceType::Response> res) {
  res->device_type = std::string(device_->getDeviceType());
  res->string_id = device_->getStringID();
}

void AstraDriver::getIRGainCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::GetIRGain::Request> /*req*/,
    std::shared_ptr<astra_msgs::srv::GetIRGain::Response> res) {
  res->gain = device_->getIRGain();
}

void AstraDriver::setIRGainCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetIRGain::Request> req,
    std::shared_ptr<astra_msgs::srv::SetIRGain::Response> res) {
  device_->setIRGain(req->gain);
}

void AstraDriver::getIRExposureCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::GetIRExposure::Request> /*req*/,
    std::shared_ptr<astra_msgs::srv::GetIRExposure::Response> res) {
  res->exposure = device_->getIRExposure();
}

void AstraDriver::setIRExposureCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetIRExposure::Request> req,
    std::shared_ptr<astra_msgs::srv::SetIRExposure::Response> /*res*/) {
  device_->setIRExposure(req->exposure);
}

void AstraDriver::setIRFloodCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetIRFlood::Request> req,
    std::shared_ptr<astra_msgs::srv::SetIRFlood::Response> /*res*/) {
  device_->setIRFlood(req->enable);
}

void AstraDriver::setLaserCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetLaser::Request> req,
    std::shared_ptr<astra_msgs::srv::SetLaser::Response> /*res*/) {
  device_->setLaser(req->enable);
}

void AstraDriver::setLDPCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetLDP::Request> req,
    std::shared_ptr<astra_msgs::srv::SetLDP::Response> /*res*/) {
  device_->setLDP(req->enable);
}

void AstraDriver::setFanCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetFan::Request> req,
    std::shared_ptr<astra_msgs::srv::SetFan::Response> /*res*/) {
  device_->setFan(req->enable);
}

void AstraDriver::resetIRGainCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::ResetIRGain::Request> /*req*/,
    std::shared_ptr<astra_msgs::srv::ResetIRGain::Response> /*res*/) {
  device_->setIRGain(0x8);
}

void AstraDriver::resetIRExposureCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::ResetIRExposure::Request> /*req*/,
    std::shared_ptr<astra_msgs::srv::ResetIRExposure::Response> /*res*/) {
  device_->setIRExposure(0x419);
}

void AstraDriver::switchIRCameraCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SwitchIRCamera::Request> req,
    std::shared_ptr<astra_msgs::srv::SwitchIRCamera::Response> res) {
  if (req->camera == "left") {
    device_->switchIRCamera(0);
  } else if (req->camera == "right") {
    device_->switchIRCamera(1);
  } else {
    RCLCPP_ERROR(get_logger(), "Only support left/right");
    res->success = false;
    return;
  }

  res->success = true;
}

void AstraDriver::setDistortioncalCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetDistortioncal::Request> req,
    std::shared_ptr<astra_msgs::srv::SetDistortioncal::Response> /*res*/) {
  device_->setDistortioncal(req->enable);
}

void AstraDriver::setAeEnableCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetAeEnable::Request> req,
    std::shared_ptr<astra_msgs::srv::SetAeEnable::Response> /*res*/) {
  device_->setAeEnable(req->enable);
}

void AstraDriver::getVersionCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::GetVersion::Request> /*req*/,
    std::shared_ptr<astra_msgs::srv::GetVersion::Response> res) {
  res->version = ASTRA_ROS_VERSION_STR;
  res->core_version = ONI_VERSION_STRING;
}

void AstraDriver::setIRAutoExposureCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetAutoExposure::Request> req,
    std::shared_ptr<astra_msgs::srv::SetAutoExposure::Response> /*res*/) {
  device_->setIRAutoExposure(req->enable);
}

void AstraDriver::setColorMirrorCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetMirror::Request> req,
    std::shared_ptr<astra_msgs::srv::SetMirror::Response> /*res*/) {
  device_->setColorMirror(req->enable);
}

void AstraDriver::setDepthMirrorCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetMirror::Request> req,
    std::shared_ptr<astra_msgs::srv::SetMirror::Response> /*res*/) {
  device_->setDepthMirror(req->enable);
}

void AstraDriver::setIRMirrorCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<astra_msgs::srv::SetMirror::Request> req,
    std::shared_ptr<astra_msgs::srv::SetMirror::Response> /*res*/) {
  device_->setIRMirror(req->enable);
}

rcl_interfaces::msg::SetParametersResult AstraDriver::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  old_config_ = config_;
  for (const auto& config : parameters) {
    if (config.get_name() == "rgb_preferred" &&
        config.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      bool rgb_preferred = config.get_value<bool>();
      if (config_init_ && rgb_preferred != config_.rgb_preferred) {
        imageConnectCb();
      }

      config_.rgb_preferred = rgb_preferred;

    } else if (config.get_name() == "depth_ir_offset_x" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      config_.depth_ir_offset_x = config.get_value<double>();
      // depth_ir_offset_x_ = config.get_value<double>();
    } else if (config.get_name() == "depth_ir_offset_y" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      config_.depth_ir_offset_y = config.get_value<double>();
      // depth_ir_offset_y_ = config.get_value<double>();
    } else if (config.get_name() == "z_offset_mm" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      config_.z_offset_mm = config.get_value<double>();
      // z_offset_mm_ = config.get_value<double>();
    } else if (config.get_name() == "z_scaling" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      config_.z_scaling = config.get_value<double>();
      // z_scaling_ = config.get_value<double>();
    } else if (config.get_name() == "ir_time_offset" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      config_.ir_time_offset = config.get_value<double>();
      ir_time_offset_ = rclcpp::Duration::from_seconds(config_.ir_time_offset);
    } else if (config.get_name() == "color_time_offset" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      config_.color_time_offset = config.get_value<double>();
      color_time_offset_ =
          rclcpp::Duration::from_seconds(config_.color_time_offset);
    } else if (config.get_name() == "depth_time_offset" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      config_.depth_time_offset = config.get_value<double>();
      depth_time_offset_ =
          rclcpp::Duration::from_seconds(config_.depth_time_offset);
    } else if (config.get_name() == "color_depth_synchronization" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      config_.color_depth_synchronization = config.get_value<bool>();
    } else if (config.get_name() == "depth_registration" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      config_.depth_registration = config.get_value<bool>();
    } else if (config.get_name() == "auto_white_balance" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      config_.auto_white_balance = config.get_value<bool>();
    } else if (config.get_name() == "use_device_time" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      config_.use_device_time = config.get_value<bool>();
    } else if (config.get_name() == "data_skip" &&
               config.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      config_.data_skip = config.get_value<int>() + 1;
    }
  }

  if (lookupVideoModeFromDynConfig(config_.ir_mode, ir_video_mode_) < 0) {
    RCLCPP_ERROR(get_logger(),
                 "Undefined IR video mode received from dynamic reconfigure");
    exit(-1);
  }

  if (lookupVideoModeFromDynConfig(config_.color_mode, color_video_mode_) < 0) {
    RCLCPP_ERROR(
        get_logger(),
        "Undefined color video mode received from dynamic reconfigure");
    exit(-1);
  }

  if (lookupVideoModeFromDynConfig(config_.depth_mode, depth_video_mode_) < 0) {
    RCLCPP_ERROR(
        get_logger(),
        "Undefined depth video mode received from dynamic reconfigure");
    exit(-1);
  }

  ir_video_mode_.pixel_format_ = PIXEL_FORMAT_GRAY16;
  color_video_mode_.pixel_format_ = PIXEL_FORMAT_RGB888;
  depth_video_mode_.pixel_format_ = PIXEL_FORMAT_DEPTH_1_MM;

  RCLCPP_INFO_STREAM(get_logger(), "depth_mode is " << depth_video_mode_);
  RCLCPP_INFO_STREAM(get_logger(), "color mode is " << color_video_mode_);
  RCLCPP_INFO_STREAM(get_logger(), "IR mode is " << ir_video_mode_);

  applyConfigToOpenNIDevice();

  config_init_ = true;

  return result;
}

void AstraDriver::setIRVideoMode(const AstraVideoMode& ir_video_mode) {
  if (device_->isIRVideoModeSupported(ir_video_mode)) {
    if (ir_video_mode != device_->getIRVideoMode()) {
      device_->setIRVideoMode(ir_video_mode);
    }
  } else {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Unsupported IR video mode - " << ir_video_mode);
  }
}
void AstraDriver::setColorVideoMode(const AstraVideoMode& color_video_mode) {
  if (device_->isColorVideoModeSupported(color_video_mode)) {
    if (color_video_mode != device_->getColorVideoMode()) {
      device_->setColorVideoMode(color_video_mode);
    }
  } else {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Unsupported color video mode - " << color_video_mode);
  }
}
void AstraDriver::setDepthVideoMode(const AstraVideoMode& depth_video_mode) {
  if (device_->isDepthVideoModeSupported(depth_video_mode)) {
    if (depth_video_mode != device_->getDepthVideoMode()) {
      device_->setDepthVideoMode(depth_video_mode);
    }
  } else {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Unsupported depth video mode - " << depth_video_mode);
  }
}

void AstraDriver::applyConfigToOpenNIDevice() {
  data_skip_ir_counter_ = 0;
  data_skip_color_counter_ = 0;
  data_skip_depth_counter_ = 0;

  setIRVideoMode(ir_video_mode_);
  if (device_->hasColorSensor()) {
    setColorVideoMode(color_video_mode_);
  }
  setDepthVideoMode(depth_video_mode_);

  if (device_->isImageRegistrationModeSupported()) {
    try {
      if (!config_init_ ||
          (old_config_.depth_registration != config_.depth_registration)) {
        // if (!config_init_)
        device_->setImageRegistrationMode(config_.depth_registration);
      }
    } catch (const AstraException& exception) {
      RCLCPP_ERROR(get_logger(), "Could not set image registration. Reason: %s",
                   exception.what());
    }
  }

  try {
    if (!config_init_ || (old_config_.color_depth_synchronization !=
                          config_.color_depth_synchronization)) {
      // if (!config_init_)
      device_->setDepthColorSync(config_.color_depth_synchronization);
    }
  } catch (const AstraException& exception) {
    RCLCPP_ERROR(get_logger(),
                 "Could not set color depth synchronization. Reason: %s",
                 exception.what());
  }

  device_->setUseDeviceTimer(config_.use_device_time);

  device_->setKeepAlive(config_.keep_alive);
}

void AstraDriver::imageConnectCb() {
  std::lock_guard<std::recursive_mutex> lock(connect_mutex_);

  bool ir_started = device_->isIRStreamStarted();
  bool color_started = device_->isColorStreamStarted();

  // ir_subscribers_ = pub_ir_.getNumSubscribers() > 0;
  // color_subscribers_ = pub_color_.getNumSubscribers() > 0;

  ir_subscribers_ = true;
  color_subscribers_ = false;

  if (color_subscribers_ && (!ir_subscribers_ || config_.rgb_preferred)) {
    if (ir_subscribers_)
      RCLCPP_ERROR(
          get_logger(),
          "Cannot stream RGB and IR at the same time. Streaming RGB only.");

    if (ir_started) {
      RCLCPP_INFO(get_logger(), "Stopping IR stream.");
      device_->stopIRStream();
    }

    if (!color_started) {
      device_->setColorFrameCallback(std::bind(
          &AstraDriver::newColorFrameCallback, this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(), "Starting color stream.");
      device_->startColorStream();
    }
  } else if (ir_subscribers_ &&
             (!color_subscribers_ || !config_.rgb_preferred)) {
    if (color_subscribers_)
      RCLCPP_ERROR(
          get_logger(),
          "Cannot stream RGB and IR at the same time. Streaming IR only.");

    if (color_started) {
      RCLCPP_INFO(get_logger(), "Stopping color stream.");
      device_->stopColorStream();
    }

    if (!ir_started) {
      device_->setIRFrameCallback(std::bind(&AstraDriver::newIRFrameCallback,
                                            this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(), "Starting IR stream.");
      device_->startIRStream();
    }
  } else {
    if (color_started) {
      RCLCPP_INFO(get_logger(), "Stopping color stream.");
      device_->stopColorStream();
    }
    if (ir_started) {
      RCLCPP_INFO(get_logger(), "Stopping IR stream.");
      device_->stopIRStream();
    }
  }
}

void AstraDriver::depthConnectCb() {
  // boost::lock_guard<boost::mutex> lock(connect_mutex_);
  std::lock_guard<std::recursive_mutex> lock(connect_mutex_);

  depth_subscribers_ = pub_depth_.getNumSubscribers() > 0;
  depth_raw_subscribers_ = pub_depth_raw_.getNumSubscribers() > 0;
  projector_info_subscribers_ =
      pub_projector_info_->get_subscription_count() > 0;

  bool need_depth = true;
  // bool need_depth = depth_subscribers_ || depth_raw_subscribers_;

  if (need_depth && !device_->isDepthStreamStarted()) {
    device_->setDepthFrameCallback(std::bind(
        &AstraDriver::newDepthFrameCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Starting depth stream.");
    device_->startDepthStream();
  } else if (!need_depth && device_->isDepthStreamStarted()) {
    RCLCPP_INFO(get_logger(), "Stopping depth stream.");
    device_->stopDepthStream();
  }
}

void AstraDriver::newIRFrameCallback(sensor_msgs::msg::Image::SharedPtr image) {
  if ((++data_skip_ir_counter_) % config_.data_skip == 0) {
    data_skip_ir_counter_ = 0;

    // TODO: Only if it has subscribers, it will publish data

    image->header.frame_id = ir_frame_id_;
    image->header.stamp = rclcpp::Time(image->header.stamp) + ir_time_offset_;

    pub_ir_.publish(image, getIRCameraInfo(image->width, image->height,
                                           image->header.stamp));
  }
}

void AstraDriver::newColorFrameCallback(
    sensor_msgs::msg::Image::SharedPtr image) {
  if ((++data_skip_color_counter_) % config_.data_skip == 0) {
    data_skip_color_counter_ = 0;

    // TODO: Only if it has subscribers, it will publish data

    image->header.frame_id = color_frame_id_;
    image->header.stamp =
        rclcpp::Time(image->header.stamp) + color_time_offset_;

    pub_color_.publish(image, getColorCameraInfo(image->width, image->height,
                                                 image->header.stamp));
  }
}

void AstraDriver::newDepthFrameCallback(
    sensor_msgs::msg::Image::SharedPtr image) {
  if ((++data_skip_depth_counter_) % config_.data_skip == 0) {
    data_skip_depth_counter_ = 0;

    // TODO: Only if it has subscribers, it will publish data

    image->header.stamp =
        rclcpp::Time(image->header.stamp) + depth_time_offset_;

    if (config_.z_offset_mm != 0) {
      uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
      for (unsigned int i = 0; i < image->width * image->height; ++i)
        if (data[i] != 0) data[i] += config_.z_offset_mm;
    }

    if (fabs(config_.z_scaling - 1.0) > 1e-6) {
      uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
      for (unsigned int i = 0; i < image->width * image->height; ++i)
        if (data[i] != 0)
          data[i] = static_cast<uint16_t>(data[i] * config_.z_scaling);
    }

    sensor_msgs::msg::CameraInfo::SharedPtr cam_info;

    if (config_.depth_registration) {
      image->header.frame_id = color_frame_id_;
    } else {
      image->header.frame_id = depth_frame_id_;
    }
    cam_info =
        getDepthCameraInfo(image->width, image->height, image->header.stamp);

    pub_depth_raw_.publish(image, cam_info);

    {
      sensor_msgs::msg::Image::ConstSharedPtr floating_point_image =
          rawToFloatingPointConversion(image);
      pub_depth_.publish(floating_point_image, cam_info);
    }

    // Projector "info" probably only useful for working with disparity images
    {
      pub_projector_info_->publish(*getProjectorCameraInfo(
          image->width, image->height, image->header.stamp));
    }
  }
}

sensor_msgs::msg::CameraInfo AstraDriver::convertAstraCameraInfo(
    OBCameraParams p, rclcpp::Time time) const {
  sensor_msgs::msg::CameraInfo info;
  // info.width = width;
  // info.height = height;
  info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  info.d.resize(5, 0.0);
  info.d[0] = p.r_k[0];
  info.d[1] = p.r_k[1];
  info.d[2] = p.r_k[3];
  info.d[3] = p.r_k[4];
  info.d[4] = p.r_k[2];

  info.k.fill(0.0);
  info.k[0] = p.r_intr_p[0];
  info.k[2] = p.r_intr_p[2];
  info.k[4] = p.r_intr_p[1];
  info.k[5] = p.r_intr_p[3];
  info.k[8] = 1.0;

  info.r.fill(0.0);
  for (int i = 0; i < 9; i++) {
    info.r[i] = p.r2l_r[i];
  }

  info.p.fill(0.0);
  info.p[0] = info.k[0];
  info.p[2] = info.k[2];
  info.p[3] = p.r2l_t[0];
  info.p[5] = info.k[4];
  info.p[6] = info.k[5];
  info.p[7] = p.r2l_t[1];
  info.p[10] = 1.0;
  info.p[11] = p.r2l_t[2];
  // Fill in header
  info.header.stamp = time;
  info.header.frame_id = color_frame_id_;
  return info;
}

// Methods to get calibration parameters for the various cameras
sensor_msgs::msg::CameraInfo::SharedPtr AstraDriver::getDefaultCameraInfo(
    int width, int height, double f) const {
  sensor_msgs::msg::CameraInfo::SharedPtr info =
      std::make_shared<sensor_msgs::msg::CameraInfo>();

  info->width = width;
  info->height = height;

  // No distortion
  info->d.resize(5, 0.0);
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // Simple camera matrix: square pixels (fx = fy), principal point at center
  info->k.fill(0.0);
  info->k[0] = info->k[4] = f;
  info->k[2] = (width / 2) - 0.5;
  // Aspect ratio for the camera center on Astra (and other devices?) is 4/3
  // This formula keeps the principal point the same in VGA and SXGA modes
  info->k[5] = (width * (3. / 8.)) - 0.5;
  info->k[8] = 1.0;

  // No separate rectified image plane, so R = I
  info->r.fill(0.0);
  info->r[0] = info->r[4] = info->r[8] = 1.0;

  // Then P=K(I|0) = (K|0)
  info->p.fill(0.0);
  info->p[0] = info->p[5] = f;  // fx, fy
  info->p[2] = info->k[2];      // cx
  info->p[6] = info->k[5];      // cy
  info->p[10] = 1.0;

  return info;
}

/// @todo Use binning/ROI properly in publishing camera infos
sensor_msgs::msg::CameraInfo::SharedPtr AstraDriver::getColorCameraInfo(
    int width, int height, rclcpp::Time time) const {
  sensor_msgs::msg::CameraInfo::SharedPtr info;

  if (color_info_manager_->isCalibrated()) {
    info = std::make_shared<sensor_msgs::msg::CameraInfo>(
        color_info_manager_->getCameraInfo());
    if (info->width != width) {
      // Use uncalibrated values
      RCLCPP_WARN_ONCE(
          get_logger(),
          "Image resolution doesn't match calibration of the RGB camera. Using "
          "default parameters.");
      info = getDefaultCameraInfo(width, height,
                                  device_->getColorFocalLength(height));
    }
  } else {
    // If uncalibrated, fill in default values
    if (device_->isCameraParamsValid()) {
      sensor_msgs::msg::CameraInfo cinfo =
          convertAstraCameraInfo(device_->getCameraParams(), time);
      info = std::make_shared<sensor_msgs::msg::CameraInfo>(
          ir_info_manager_->getCameraInfo());
      info->d.resize(5, 0.0);
      info->k.fill(0.0);
      info->r.fill(0.0);
      info->p.fill(0.0);
      info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      info->width = width;
      info->height = height;

      for (int i = 0; i < 9; i++) {
        info->k[i] = cinfo.k[i];
        info->r[i] = cinfo.r[i];
      }

      for (int i = 0; i < 12; i++) {
        info->p[i] = cinfo.p[i];
      }
      /*02112020 color camera param change according to resolution */
      double scaling = (double)width / 640;
      info->k[0] *= scaling;  // fx
      info->k[2] *= scaling;  // cx
      info->k[4] *= scaling;  // fy
      info->k[5] *= scaling;  // cy
      info->p[0] *= scaling;  // fx
      info->p[2] *= scaling;  // cx
      info->p[5] *= scaling;  // fy
      info->p[6] *= scaling;  // cy
                              /* 02112020 end*/

    } else {
      info = getDefaultCameraInfo(width, height,
                                  device_->getColorFocalLength(height));
    }
  }

  // Fill in header
  info->header.stamp = time;
  info->header.frame_id = color_frame_id_;

  return info;
}

sensor_msgs::msg::CameraInfo::SharedPtr AstraDriver::getIRCameraInfo(
    int width, int height, rclcpp::Time time) const {
  sensor_msgs::msg::CameraInfo::SharedPtr info;

  if (ir_info_manager_->isCalibrated()) {
    info = std::make_shared<sensor_msgs::msg::CameraInfo>(
        ir_info_manager_->getCameraInfo());
    if (info->width != width) {
      // Use uncalibrated values
      RCLCPP_WARN_ONCE(
          get_logger(),
          "Image resolution doesn't match calibration of the IR camera. Using "
          "default parameters.");
      info = getDefaultCameraInfo(width, height,
                                  device_->getIRFocalLength(height));
    }
  } else {
    // If uncalibrated, fill in default values
    info = getDefaultCameraInfo(width, height,
                                device_->getDepthFocalLength(height));

    if (device_->isCameraParamsValid()) {
      OBCameraParams p = device_->getCameraParams();
      info->d.resize(5, 0.0);
      // info->D[0] = p.l_k[0];
      // info->D[1] = p.l_k[1];
      // info->D[2] = p.l_k[3];
      // info->D[3] = p.l_k[4];
      // info->D[4] = p.l_k[2];

      info->k.fill(0.0);
      info->k[0] = p.l_intr_p[0];
      info->k[2] = p.l_intr_p[2];
      info->k[4] = p.l_intr_p[1];
      info->k[5] = p.l_intr_p[3];
      info->k[8] = 1.0;

      info->r.fill(0.0);
      info->r[0] = 1;
      info->r[4] = 1;
      info->r[8] = 1;

      info->p.fill(0.0);
      info->p[0] = info->k[0];
      info->p[2] = info->k[2];
      info->p[5] = info->k[4];
      info->p[6] = info->k[5];
      info->p[10] = 1.0;
      /* 02122020 Scale IR Params */
      double scaling = (double)width / 640;
      info->k[0] *= scaling;  // fx
      info->k[2] *= scaling;  // cx
      info->k[4] *= scaling;  // fy
      info->k[5] *= scaling;  // cy
      info->p[0] *= scaling;  // fx
      info->p[2] *= scaling;  // cx
      info->p[5] *= scaling;  // fy
      info->p[6] *= scaling;  // cy
                              /* 02122020 end */
    }
  }

  // Fill in header
  info->header.stamp = time;
  info->header.frame_id = depth_frame_id_;

  return info;
}

sensor_msgs::msg::CameraInfo::SharedPtr AstraDriver::getDepthCameraInfo(
    int width, int height, rclcpp::Time time) const {
  // The depth image has essentially the same intrinsics as the IR image, BUT
  // the principal point is offset by half the size of the hardware correlation
  // window (probably 9x9 or 9x7 in 640x480 mode). See
  // http://www.ros.org/wiki/kinect_calibration/technical
  double scaling = (double)width / 640;
  sensor_msgs::msg::CameraInfo::SharedPtr info =
      getIRCameraInfo(width, height, time);
  info->k[2] -= config_.depth_ir_offset_x * scaling;
  info->k[5] -= config_.depth_ir_offset_y * scaling;
  info->p[2] -= config_.depth_ir_offset_x * scaling;
  info->p[6] -= config_.depth_ir_offset_y * scaling;

  /// @todo Could put this in projector frame so as to encode the baseline in
  /// P[3]
  return info;
}

sensor_msgs::msg::CameraInfo::SharedPtr AstraDriver::getProjectorCameraInfo(
    int width, int height, rclcpp::Time time) const {
  // The projector info is simply the depth info with the baseline encoded in
  // the P matrix. It's only purpose is to be the "right" camera info to the
  // depth camera's "left" for processing disparity images.
  sensor_msgs::msg::CameraInfo::SharedPtr info =
      getDepthCameraInfo(width, height, time);
  // Tx = -baseline * fx
  info->p[3] = -device_->getBaseline() * info->p[0];
  return info;
}

void AstraDriver::initParameters() {
  config_.rgb_preferred = this->declare_parameter<bool>("rgb_preferred", false);
  config_.color_mode = this->declare_parameter<int>("color_mode", 7);
  config_.depth_mode = this->declare_parameter<int>("depth_mode", 7);
  config_.ir_mode = this->declare_parameter<int>("ir_mode", 7);
  config_.depth_registration =
      this->declare_parameter<bool>("depth_registration", true);
  config_.color_depth_synchronization =
      this->declare_parameter<bool>("color_depth_synchronization", false);
  // config_.auto_exposure = this->declare_parameter<bool>("auto_exposure",
  // true);
  config_.auto_white_balance =
      this->declare_parameter<bool>("auto_white_balance", true);
  config_.data_skip = this->declare_parameter<int>("data_skip", 0);
  config_.data_skip += 1;
  config_.ir_time_offset =
      this->declare_parameter<double>("ir_time_offset", -0.033);
  config_.color_time_offset =
      this->declare_parameter<double>("color_time_offset", -0.033);
  config_.depth_time_offset =
      this->declare_parameter<double>("depth_time_offset", -0.033);
  ;
  config_.depth_ir_offset_x =
      this->declare_parameter<double>("depth_ir_offset_x", 5.0);
  ;
  config_.depth_ir_offset_y =
      this->declare_parameter<double>("depth_ir_offset_y", 4.0);
  ;
  config_.z_offset_mm = this->declare_parameter<double>("z_offset_mm", 0.0);
  config_.z_scaling = this->declare_parameter<double>("z_scaling", 1.0);
  config_.use_device_time =
      this->declare_parameter<bool>("use_device_time", true);
  config_.keep_alive = this->declare_parameter<bool>("keep_alive", false);

  ir_frame_id_ = this->declare_parameter<std::string>(
      "ir_frame_id", "openni_rgb_optical_frame");
  color_frame_id_ = this->declare_parameter<std::string>(
      "rgb_frame_id", "openni_ir_optical_frame");
  depth_frame_id_ = this->declare_parameter<std::string>(
      "depth_frame_id", "openni_depth_optical_frame");

  RCLCPP_INFO(get_logger(), "ir_frame_id = '%s' ", ir_frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "rgb_frame_id = '%s' ", color_frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "depth_frame_id = '%s' ", depth_frame_id_.c_str());

  color_info_url_ =
      this->declare_parameter<std::string>("rgb_camera_info_url", "");
  ir_info_url_ =
      this->declare_parameter<std::string>("depth_camera_info_url", "");
}

void AstraDriver::initServices() {
  get_camera_info_srv_ = create_service<astra_msgs::srv::GetCameraInfo>(
      "get_camera_info", std::bind(&AstraDriver::getCameraInfoCallback, this,
                                   std::placeholders::_1, std::placeholders::_2,
                                   std::placeholders::_3));
  get_serial_srv_ = create_service<astra_msgs::srv::GetSerial>(
      "get_serial",
      std::bind(&AstraDriver::getSerialCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  get_device_type_srv_ = create_service<astra_msgs::srv::GetDeviceType>(
      "get_device_type", std::bind(&AstraDriver::getDeviceTypeCallback, this,
                                   std::placeholders::_1, std::placeholders::_2,
                                   std::placeholders::_3));
  get_ir_gain_srv_ = create_service<astra_msgs::srv::GetIRGain>(
      "get_ir_gain",
      std::bind(&AstraDriver::getIRGainCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  set_ir_gain_srv_ = create_service<astra_msgs::srv::SetIRGain>(
      "set_ir_gain",
      std::bind(&AstraDriver::setIRGainCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  get_ir_exposure_srv_ = create_service<astra_msgs::srv::GetIRExposure>(
      "get_ir_exposure", std::bind(&AstraDriver::getIRExposureCallback, this,
                                   std::placeholders::_1, std::placeholders::_2,
                                   std::placeholders::_3));
  set_ir_exposure_srv_ = create_service<astra_msgs::srv::SetIRExposure>(
      "set_ir_exposure", std::bind(&AstraDriver::setIRExposureCallback, this,
                                   std::placeholders::_1, std::placeholders::_2,
                                   std::placeholders::_3));
  set_ir_flood_srv_ = create_service<astra_msgs::srv::SetIRFlood>(
      "set_ir_flood",
      std::bind(&AstraDriver::setIRFloodCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  set_laser_srv_ = create_service<astra_msgs::srv::SetLaser>(
      "set_laser",
      std::bind(&AstraDriver::setLaserCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  set_ldp_srv_ = create_service<astra_msgs::srv::SetLDP>(
      "set_ldp",
      std::bind(&AstraDriver::setLDPCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  set_fan_srv_ = create_service<astra_msgs::srv::SetFan>(
      "set_fan",
      std::bind(&AstraDriver::setFanCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  reset_ir_gain_srv_ = create_service<astra_msgs::srv::ResetIRGain>(
      "reset_ir_gain",
      std::bind(&AstraDriver::resetIRGainCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  reset_ir_exposure_srv_ = create_service<astra_msgs::srv::ResetIRExposure>(
      "reset_ir_exposure",
      std::bind(&AstraDriver::resetIRExposureCallback, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));
  get_version_srv_ = create_service<astra_msgs::srv::GetVersion>(
      "get_version",
      std::bind(&AstraDriver::getVersionCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  set_ir_auto_exposure_srv_ = create_service<astra_msgs::srv::SetAutoExposure>(
      "set_ir_auto_exposure",
      std::bind(&AstraDriver::setIRAutoExposureCallback, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));
  set_color_mirror_srv_ = create_service<astra_msgs::srv::SetMirror>(
      "set_color_mirror",
      std::bind(&AstraDriver::setColorMirrorCallback, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));
  set_depth_mirror_srv_ = create_service<astra_msgs::srv::SetMirror>(
      "set_depth_mirror",
      std::bind(&AstraDriver::setDepthMirrorCallback, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));
  set_ir_mirror_srv_ = create_service<astra_msgs::srv::SetMirror>(
      "set_ir_mirror",
      std::bind(&AstraDriver::setIRMirrorCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  if (device_->getDeviceTypeNo() == OB_STEREO_S_NO ||
      device_->getDeviceTypeNo() == OB_STEREO_S_U3_NO ||
      device_->getDeviceTypeNo() == OB_DABAI_NO ||
      device_->getDeviceTypeNo() == OB_DABAI_PRO_NO) {
    switch_ir_camera_srv_ = create_service<astra_msgs::srv::SwitchIRCamera>(
        "switch_ir_camera",
        std::bind(&AstraDriver::switchIRCameraCallback, this,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3));
  }

  if (device_->getDeviceTypeNo() == OB_STEREO_S_NO ||
      device_->getDeviceTypeNo() == OB_STEREO_S_U3_NO) {
    set_distortioncal_srv_ = create_service<astra_msgs::srv::SetDistortioncal>(
        "set_distortioncal",
        std::bind(&AstraDriver::setDistortioncalCallback, this,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3));
    set_ae_enable_srv_ = create_service<astra_msgs::srv::SetAeEnable>(
        "set_ae_enable", std::bind(&AstraDriver::setAeEnableCallback, this,
                                   std::placeholders::_1, std::placeholders::_2,
                                   std::placeholders::_3));
  }
}

std::string AstraDriver::resolveDeviceURI(const std::string& device_id) {
  // retrieve available device URIs, they look like this: "1d27/0601@1/5"
  // which is <vendor ID>/<product ID>@<bus number>/<device number>
  std::shared_ptr<std::vector<std::string> > available_device_URIs =
      device_manager_->getConnectedDeviceURIs();

// for tes
#if 0
   for (size_t i = 0; i < available_device_URIs->size(); ++i)
   {
       std::string s = (*available_device_URIs)[i];
  	RCLCPP_WARN(get_logger(), "------------id %d, available_device_uri is %s-----------", i, s.c_str());
   }
#endif
  // end
  // look for '#<number>' format
  if (device_id.size() > 1 && device_id[0] == '#') {
    std::istringstream device_number_str(device_id.substr(1));
    int device_number;
    device_number_str >> device_number;
    int device_index = device_number - 1;  // #1 refers to first device
    if (device_index >= available_device_URIs->size() || device_index < 0) {
      THROW_OPENNI_EXCEPTION(
          "Invalid device number %i, there are %zu devices connected.",
          device_number, available_device_URIs->size());
    } else {
      return available_device_URIs->at(device_index);
    }
  }
  // look for '<bus>@<number>' format
  //   <bus>    is usb bus id, typically start at 1
  //   <number> is the device number, for consistency with astra_camera, these
  //   start at 1
  //               although 0 specifies "any device on this bus"
  else if (device_id.size() > 1 && device_id.find('@') != std::string::npos &&
           device_id.find('/') == std::string::npos) {
    // get index of @ character
    size_t index = device_id.find('@');
    if (index <= 0) {
      THROW_OPENNI_EXCEPTION(
          "%s is not a valid device URI, you must give the bus number before "
          "the @.",
          device_id.c_str());
    }
    if (index >= device_id.size() - 1) {
      THROW_OPENNI_EXCEPTION(
          "%s is not a valid device URI, you must give a number after the @, "
          "specify 0 for first device",
          device_id.c_str());
    }

    // pull out device number on bus
    std::istringstream device_number_str(device_id.substr(index + 1));
    int device_number;
    device_number_str >> device_number;

    // reorder to @<bus>
    std::string bus = device_id.substr(0, index);
    bus.insert(0, "@");

    for (size_t i = 0; i < available_device_URIs->size(); ++i) {
      std::string s = (*available_device_URIs)[i];
      if (s.find(bus) != std::string::npos) {
        // this matches our bus, check device number
        --device_number;
        if (device_number <= 0) return s;
      }
    }

    THROW_OPENNI_EXCEPTION("Device not found %s", device_id.c_str());
  } else {
    // check if the device id given matches a serial number of a connected
    // device
    for (std::vector<std::string>::const_iterator it =
             available_device_URIs->begin();
         it != available_device_URIs->end(); ++it) {
#if 0
      	try 
	{
        	std::string serial = device_manager_->getSerial(*it);
        	if (serial.size() > 0 && device_id == serial)
          		return *it;
	}
#else
      try {
        std::set<std::string>::iterator iter;
        if ((iter = alreadyOpen.find(*it)) == alreadyOpen.end()) {
          // RCLCPP_WARN(get_logger(), "------------seraial num it is  %s,
          // device_id is %s
          // -----------", (*it).c_str(), device_id_.c_str());
          std::string serial = device_manager_->getSerial(*it);
          if (serial.size() > 0 && device_id == serial) {
            alreadyOpen.insert(*it);
            return *it;
          }
        }
      }
#endif
      catch (const AstraException& exception) {
        RCLCPP_WARN(get_logger(),
                    "Could not query serial number of device \"%s\":",
                    exception.what());
      }
    }

    // everything else is treated as part of the device_URI
    bool match_found = false;
    std::string matched_uri;
    for (size_t i = 0; i < available_device_URIs->size(); ++i) {
      std::string s = (*available_device_URIs)[i];
      if (s.find(device_id) != std::string::npos) {
        if (!match_found) {
          matched_uri = s;
          match_found = true;
        } else {
          // more than one match
          THROW_OPENNI_EXCEPTION(
              "Two devices match the given device id '%s': %s and %s.",
              device_id.c_str(), matched_uri.c_str(), s.c_str());
        }
      }
    }
    return matched_uri;
  }

  return "INVALID";
}

void AstraDriver::initDevice() {
  RCLCPP_INFO(get_logger(), "initDevice");

  while (rclcpp::ok() && !device_) {
    try {
      std::string device_URI = resolveDeviceURI(device_id_);
#if 0
      if( device_URI == "" ) 
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      	continue;
      }
#endif
      device_ = device_manager_->getDevice(device_URI);
    } catch (const AstraException& exception) {
      if (!device_) {
        RCLCPP_INFO(
            get_logger(),
            "No matching device found.... waiting for devices. Reason: %s",
            exception.what());
        std::this_thread::sleep_for(std::chrono::seconds(3));
        continue;
      } else {
        RCLCPP_ERROR(get_logger(), "Could not retrieve device. Reason: %s",
                     exception.what());
        exit(-1);
      }
    }
  }

  while (rclcpp::ok() && !device_->isValid()) {
    RCLCPP_DEBUG(get_logger(), "Waiting for device initialization..");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void AstraDriver::genVideoModeTableMap() {
  video_modes_lookup_.clear();

  AstraVideoMode video_mode;

  // SXGA_30Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 1024;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[1] = video_mode;

  // SXGA_15Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 1024;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[2] = video_mode;

  // 1280x800_30Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 800;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[3] = video_mode;

  // 1280x800_15Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 800;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[4] = video_mode;

  // XGA_30Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 720;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[5] = video_mode;

  // XGA_15Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 720;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[6] = video_mode;

  // VGA_30Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[7] = video_mode;

  // VGA_15Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[8] = video_mode;

  // VGA_60Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[9] = video_mode;

  // QVGA_30Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[10] = video_mode;

  // QVGA_15Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[11] = video_mode;

  // QVGA_60Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[12] = video_mode;

  // QQVGA_30Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[13] = video_mode;

  // QQVGA_15Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[14] = video_mode;

  // QQVGA_60Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[15] = video_mode;

  // 640*400_30Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 400;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[16] = video_mode;

  // 640*400_15Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 400;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[17] = video_mode;

  // 640*400_10Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 400;
  video_mode.frame_rate_ = 10;

  video_modes_lookup_[18] = video_mode;

  // 640*400_5Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 400;
  video_mode.frame_rate_ = 5;

  video_modes_lookup_[19] = video_mode;

  // 640*400_60Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 400;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[20] = video_mode;

  // 320*200_30Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 200;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[21] = video_mode;

  // 320*200_15Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 200;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[22] = video_mode;

  // 320*200_10Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 200;
  video_mode.frame_rate_ = 10;

  video_modes_lookup_[23] = video_mode;

  // 320*200_5Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 200;
  video_mode.frame_rate_ = 5;

  video_modes_lookup_[24] = video_mode;

  // 1280*1024_7Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 800;
  video_mode.frame_rate_ = 7;

  video_modes_lookup_[25] = video_mode;

  // 1280*800_7Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 800;
  video_mode.frame_rate_ = 7;

  video_modes_lookup_[26] = video_mode;

  // 320*200_60Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 200;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[27] = video_mode;

  // 320*240_60Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[28] = video_mode;
}

int AstraDriver::lookupVideoModeFromDynConfig(int mode_nr,
                                              AstraVideoMode& video_mode) {
  int ret = -1;

  std::map<int, AstraVideoMode>::const_iterator it;

  it = video_modes_lookup_.find(mode_nr);

  if (it != video_modes_lookup_.end()) {
    video_mode = it->second;
    ret = 0;
  }

  return ret;
}

sensor_msgs::msg::Image::ConstSharedPtr
AstraDriver::rawToFloatingPointConversion(
    sensor_msgs::msg::Image::ConstSharedPtr raw_image) {
  static const float bad_point = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::msg::Image::SharedPtr new_image =
      std::make_shared<sensor_msgs::msg::Image>();

  new_image->header = raw_image->header;
  new_image->width = raw_image->width;
  new_image->height = raw_image->height;
  new_image->is_bigendian = 0;
  new_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  new_image->step = sizeof(float) * raw_image->width;

  std::size_t data_size = new_image->width * new_image->height;
  new_image->data.resize(data_size * sizeof(float));

  const unsigned short* in_ptr =
      reinterpret_cast<const unsigned short*>(&raw_image->data[0]);
  float* out_ptr = reinterpret_cast<float*>(&new_image->data[0]);

  for (std::size_t i = 0; i < data_size; ++i, ++in_ptr, ++out_ptr) {
    if (*in_ptr == 0 || *in_ptr == 0x7FF) {
      *out_ptr = bad_point;
    } else {
      *out_ptr = static_cast<float>(*in_ptr) / 1000.0f;
    }
  }

  return new_image;
}

}  // namespace ros2_astra_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_astra_camera::AstraDriver)
