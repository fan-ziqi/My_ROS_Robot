/*
 * Author: Kaven Yau (kavenyau@foxmail.com)
 */

#ifndef CAMERA_DRIVER_H_
#define CAMERA_DRIVER_H_

#include <libuvc/libuvc.h>
#include <ros2_astra_camera/astra_wrapper/astra_device_type.h>

#include <astra_msgs/srv/get_camera_info.hpp>
#include <astra_msgs/srv/get_device_type.hpp>
#include <astra_msgs/srv/get_uvc_exposure.hpp>
#include <astra_msgs/srv/get_uvc_gain.hpp>
#include <astra_msgs/srv/get_uvc_white_balance.hpp>
#include <astra_msgs/srv/set_auto_exposure.hpp>
#include <astra_msgs/srv/set_auto_white_balance.hpp>
#include <astra_msgs/srv/set_uvc_exposure.hpp>
#include <astra_msgs/srv/set_uvc_gain.hpp>
#include <astra_msgs/srv/set_uvc_white_balance.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>

#include "uvc_camera_config.h"

namespace ros2_astra_camera {

class CameraDriver : public rclcpp::Node {
 public:
  CameraDriver(const rclcpp::NodeOptions &options);
  ~CameraDriver();

  bool Start();
  void Stop();

 private:
  enum State {
    kInitial = 0,
    kStopped = 1,
    kRunning = 2,
  };

  // Flags controlling whether the sensor needs to be stopped (or reopened) when
  // changing settings
  static const int kReconfigureClose =
      3;  // Need to close and reopen sensor to change this setting
  static const int kReconfigureStop =
      1;  // Need to stop the stream before changing this setting
  static const int kReconfigureRunning =
      0;  // We can change this setting without stopping the stream

  void initParameters();
  void initServices();
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
      std::vector<rclcpp::Parameter> parameters);

  void OpenCamera(UVCCameraConfig &new_config);
  void CloseCamera();

  enum uvc_frame_format GetVideoMode(std::string vmode);
  void AutoControlsCallback(enum uvc_status_class status_class, int event,
                            int selector,
                            enum uvc_status_attribute status_attribute,
                            void *data, size_t data_len);
  static void AutoControlsCallbackAdapter(
      enum uvc_status_class status_class, int event, int selector,
      enum uvc_status_attribute status_attribute, void *data, size_t data_len,
      void *ptr);
  void ImageCallback(uvc_frame_t *frame);
  static void ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);

  void getUVCExposureCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::GetUVCExposure::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::GetUVCExposure::Response> /*res*/);
  void setUVCExposureCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetUVCExposure::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetUVCExposure::Response> /*res*/);
  void getUVCGainCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::GetUVCGain::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::GetUVCGain::Response> /*res*/);
  void setUVCGainCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetUVCGain::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetUVCGain::Response> /*res*/);
  void getUVCWhiteBalanceCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<
          astra_msgs::srv::GetUVCWhiteBalance::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::GetUVCWhiteBalance::Response> /*res*/);
  void setUVCWhiteBalanceCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<
          astra_msgs::srv::SetUVCWhiteBalance::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetUVCWhiteBalance::Response> /*res*/);
  void setAutoExposureCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<astra_msgs::srv::SetAutoExposure::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetAutoExposure::Response> /*res*/);
  void setAutoWhiteBalanceCallback(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<
          astra_msgs::srv::SetAutoWhiteBalance::Request> /*req*/,
      std::shared_ptr<astra_msgs::srv::SetAutoWhiteBalance::Response> /*res*/);

  State state_;
  std::recursive_mutex mutex_;

  uvc_context_t *ctx_;
  uvc_device_t *dev_;
  uvc_device_handle_t *devh_;
  uvc_frame_t *rgb_frame_;

  image_transport::CameraPublisher cam_pub_;

  UVCCameraConfig config_;
  UVCCameraConfig old_config_;
  bool config_changed_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      dyn_params_handler_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> color_info_manager_;

  bool param_init_;
  std::string ns;
  std::string ns_no_slash;

  rclcpp::Node::SharedPtr client_node_;

  rclcpp::Service<astra_msgs::srv::GetUVCExposure>::SharedPtr
      get_uvc_exposure_srv_;
  rclcpp::Service<astra_msgs::srv::SetUVCExposure>::SharedPtr
      set_uvc_exposure_srv_;
  rclcpp::Service<astra_msgs::srv::GetUVCGain>::SharedPtr get_uvc_gain_srv_;
  rclcpp::Service<astra_msgs::srv::SetUVCGain>::SharedPtr set_uvc_gain_srv_;
  rclcpp::Service<astra_msgs::srv::GetUVCWhiteBalance>::SharedPtr
      get_uvc_white_balance_srv_;
  rclcpp::Service<astra_msgs::srv::SetUVCWhiteBalance>::SharedPtr
      set_uvc_white_balance_srv_;
  rclcpp::Service<astra_msgs::srv::SetAutoExposure>::SharedPtr
      set_uvc_auto_exposure_srv_;
  rclcpp::Service<astra_msgs::srv::SetAutoWhiteBalance>::SharedPtr
      set_uvc_auto_white_balance_srv_;

  rclcpp::Client<astra_msgs::srv::GetDeviceType>::SharedPtr device_type_client_;
  rclcpp::Client<astra_msgs::srv::GetCameraInfo>::SharedPtr camera_info_client_;

  bool device_type_init_;
  bool camera_info_init_;
  std::string device_type_;
  sensor_msgs::msg::CameraInfo camera_info_;
  int uvc_flip_;
  OB_DEVICE_NO device_type_no_;
  bool camera_info_valid_;
};

}  // namespace ros2_astra_camera

#endif  // CAMERA_DRIVER_H_