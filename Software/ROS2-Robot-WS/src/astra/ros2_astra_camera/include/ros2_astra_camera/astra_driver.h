/*
 * Author: Kaven Yau (kavenyau@foxmail.com)
 */

#ifndef ASTRA_DRIVER_H_
#define ASTRA_DRIVER_H_

#include <astra_msgs/srv/get_camera_info.hpp>
#include <astra_msgs/srv/get_device_type.hpp>
#include <astra_msgs/srv/get_ir_exposure.hpp>
#include <astra_msgs/srv/get_ir_gain.hpp>
#include <astra_msgs/srv/get_serial.hpp>
#include <astra_msgs/srv/get_version.hpp>
#include <astra_msgs/srv/reset_ir_exposure.hpp>
#include <astra_msgs/srv/reset_ir_gain.hpp>
#include <astra_msgs/srv/set_ae_enable.hpp>
#include <astra_msgs/srv/set_auto_exposure.hpp>
#include <astra_msgs/srv/set_auto_white_balance.hpp>
#include <astra_msgs/srv/set_distortioncal.hpp>
#include <astra_msgs/srv/set_fan.hpp>
#include <astra_msgs/srv/set_ir_exposure.hpp>
#include <astra_msgs/srv/set_ir_flood.hpp>
#include <astra_msgs/srv/set_ir_gain.hpp>
#include <astra_msgs/srv/set_laser.hpp>
#include <astra_msgs/srv/set_ldp.hpp>
#include <astra_msgs/srv/set_mirror.hpp>
#include <astra_msgs/srv/switch_ir_camera.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

#include "ros2_astra_camera/astra_config.h"
#include "ros2_astra_camera/astra_wrapper/astra_device.h"
#include "ros2_astra_camera/astra_wrapper/astra_device_manager.h"
#include "ros2_astra_camera/astra_wrapper/astra_device_type.h"
#include "ros2_astra_camera/astra_wrapper/astra_video_mode.h"

namespace ros2_astra_camera {
using namespace astra_camera;
class AstraDriver : public rclcpp::Node {
 public:
  AstraDriver(const rclcpp::NodeOptions& options);
  ~AstraDriver();

 private:
  typedef astra_camera::AstraConfig AstraConfig;

  void newIRFrameCallback(sensor_msgs::msg::Image::SharedPtr image);
  void newColorFrameCallback(sensor_msgs::msg::Image::SharedPtr image);
  void newDepthFrameCallback(sensor_msgs::msg::Image::SharedPtr image);

  sensor_msgs::msg::CameraInfo::SharedPtr getDefaultCameraInfo(int width,
                                                               int height,
                                                               double f) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getColorCameraInfo(
      int width, int height, rclcpp::Time time) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getIRCameraInfo(
      int width, int height, rclcpp::Time time) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getDepthCameraInfo(
      int width, int height, rclcpp::Time time) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getProjectorCameraInfo(
      int width, int height, rclcpp::Time time) const;
  sensor_msgs::msg::CameraInfo convertAstraCameraInfo(OBCameraParams p,
                                                      rclcpp::Time time) const;

  void initParameters();
  void initServices();

  std::string resolveDeviceURI(const std::string& device_id);
  void initDevice();

  void advertiseROSTopics();

  void imageConnectCb();
  void depthConnectCb();

  void getCameraInfoCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::GetCameraInfo::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::GetCameraInfo::Response> /*res*/);
  void getSerialCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::GetSerial::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::GetSerial::Response> /*res*/);
  void getDeviceTypeCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::GetDeviceType::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::GetDeviceType::Response> /*res*/);
  void getIRGainCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::GetIRGain::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::GetIRGain::Response> /*res*/);
  void setIRGainCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetIRGain::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetIRGain::Response> /*res*/);
  void getIRExposureCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::GetIRExposure::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::GetIRExposure::Response> /*res*/);
  void setIRExposureCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetIRExposure::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetIRExposure::Response> /*res*/);
  void setIRFloodCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetIRFlood::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetIRFlood::Response> /*res*/);
  void setLaserCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetLaser::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetLaser::Response> /*res*/);
  void setLDPCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetLDP::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetLDP::Response> /*res*/);
  void setFanCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetFan::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetFan::Response> /*res*/);
  void resetIRGainCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::ResetIRGain::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::ResetIRGain::Response> /*res*/);
  void resetIRExposureCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::ResetIRExposure::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::ResetIRExposure::Response> /*res*/);
  void switchIRCameraCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SwitchIRCamera::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SwitchIRCamera::Response> /*res*/);
  void setDistortioncalCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetDistortioncal::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetDistortioncal::Response> /*res*/);
  void setAeEnableCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetAeEnable::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetAeEnable::Response> /*res*/);
  void getVersionCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::GetVersion::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::GetVersion::Response> /*res*/);
  void setIRAutoExposureCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetAutoExposure::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetAutoExposure::Response> /*res*/);
  void setColorMirrorCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetMirror::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetMirror::Response> /*res*/);
  void setDepthMirrorCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetMirror::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetMirror::Response> /*res*/);
  void setIRMirrorCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetMirror::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetMirror::Response> /*res*/);

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
      std::vector<rclcpp::Parameter> parameters);

  void applyConfigToOpenNIDevice();

  void genVideoModeTableMap();
  int lookupVideoModeFromDynConfig(int mode_nr, AstraVideoMode& video_mode);

  sensor_msgs::msg::Image::ConstSharedPtr rawToFloatingPointConversion(
      sensor_msgs::msg::Image::ConstSharedPtr raw_image);

  void setIRVideoMode(const AstraVideoMode& ir_video_mode);
  void setColorVideoMode(const AstraVideoMode& color_video_mode);
  void setDepthVideoMode(const AstraVideoMode& depth_video_mode);

  void subscribersMonitor();
  // ros::NodeHandle& nh_;
  // ros::NodeHandle& pnh_;

  std::shared_ptr<AstraDeviceManager> device_manager_;
  std::shared_ptr<AstraDevice> device_;

  std::string device_id_;

  rclcpp::Service<astra_msgs::srv::GetCameraInfo>::SharedPtr
      get_camera_info_srv_;
  rclcpp::Service<astra_msgs::srv::GetSerial>::SharedPtr get_serial_srv_;
  rclcpp::Service<astra_msgs::srv::GetDeviceType>::SharedPtr
      get_device_type_srv_;
  rclcpp::Service<astra_msgs::srv::GetIRGain>::SharedPtr get_ir_gain_srv_;
  rclcpp::Service<astra_msgs::srv::SetIRGain>::SharedPtr set_ir_gain_srv_;
  rclcpp::Service<astra_msgs::srv::GetIRExposure>::SharedPtr
      get_ir_exposure_srv_;
  rclcpp::Service<astra_msgs::srv::SetIRExposure>::SharedPtr
      set_ir_exposure_srv_;
  rclcpp::Service<astra_msgs::srv::SetIRFlood>::SharedPtr set_ir_flood_srv_;
  rclcpp::Service<astra_msgs::srv::SetLaser>::SharedPtr set_laser_srv_;
  rclcpp::Service<astra_msgs::srv::SetLDP>::SharedPtr set_ldp_srv_;
  rclcpp::Service<astra_msgs::srv::SetFan>::SharedPtr set_fan_srv_;
  rclcpp::Service<astra_msgs::srv::ResetIRGain>::SharedPtr reset_ir_gain_srv_;
  rclcpp::Service<astra_msgs::srv::ResetIRExposure>::SharedPtr
      reset_ir_exposure_srv_;
  rclcpp::Service<astra_msgs::srv::SwitchIRCamera>::SharedPtr
      switch_ir_camera_srv_;
  rclcpp::Service<astra_msgs::srv::SetDistortioncal>::SharedPtr
      set_distortioncal_srv_;
  rclcpp::Service<astra_msgs::srv::SetAeEnable>::SharedPtr set_ae_enable_srv_;
  rclcpp::Service<astra_msgs::srv::GetVersion>::SharedPtr get_version_srv_;
  rclcpp::Service<astra_msgs::srv::SetAutoExposure>::SharedPtr
      set_ir_auto_exposure_srv_;
  rclcpp::Service<astra_msgs::srv::SetMirror>::SharedPtr set_color_mirror_srv_;
  rclcpp::Service<astra_msgs::srv::SetMirror>::SharedPtr set_depth_mirror_srv_;
  rclcpp::Service<astra_msgs::srv::SetMirror>::SharedPtr set_ir_mirror_srv_;

  bool config_init_;

  std::set<std::string> alreadyOpen;
  std::recursive_mutex connect_mutex_;
  // published topics
  image_transport::CameraPublisher pub_color_;
  image_transport::CameraPublisher pub_depth_;
  image_transport::CameraPublisher pub_depth_raw_;
  image_transport::CameraPublisher pub_ir_;
  // ros::Publisher pub_projector_info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      pub_projector_info_;

  /** \brief Camera info manager objects. */
  std::shared_ptr<camera_info_manager::CameraInfoManager> color_info_manager_,
      ir_info_manager_;
  rclcpp::TimerBase::SharedPtr subscribers_monitor_timer_;

  AstraVideoMode ir_video_mode_;
  AstraVideoMode color_video_mode_;
  AstraVideoMode depth_video_mode_;

  std::string ir_frame_id_;
  std::string color_frame_id_;
  std::string depth_frame_id_;

  std::string color_info_url_, ir_info_url_;

  bool color_depth_synchronization_;
  bool depth_registration_;

  std::map<int, AstraVideoMode> video_modes_lookup_;

  AstraConfig config_;
  AstraConfig old_config_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      dyn_params_handler_;

  rclcpp::Duration ir_time_offset_;
  rclcpp::Duration color_time_offset_;
  rclcpp::Duration depth_time_offset_;
  int data_skip_ir_counter_;
  int data_skip_color_counter_;
  int data_skip_depth_counter_;
  bool ir_subscribers_;
  bool color_subscribers_;
  bool depth_subscribers_;
  bool depth_raw_subscribers_;
  bool projector_info_subscribers_;
};

}  // namespace ros2_astra_camera

#endif  // ASTRA_DRIVER_H_
