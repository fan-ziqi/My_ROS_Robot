#pragma once

#include <libuvc/libuvc.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <astra_camera/GetUVCExposure.h>
#include <astra_camera/SetUVCExposure.h>
#include <astra_camera/GetUVCGain.h>
#include <astra_camera/SetUVCGain.h>
#include <astra_camera/GetUVCWhiteBalance.h>
#include <astra_camera/SetUVCWhiteBalance.h>
#include <astra_camera/astra_device_type.h>
#include <astra_camera/UVCCameraConfig.h>

#include <string>

namespace libuvc_camera {

class CameraDriver {
public:
  CameraDriver(ros::NodeHandle nh, ros::NodeHandle priv_nh);
  ~CameraDriver();

  bool Start();
  void Stop();

private:
  enum State {
    kInitial = 0,
    kStopped = 1,
    kRunning = 2,
  };

  // Flags controlling whether the sensor needs to be stopped (or reopened) when changing settings
  static const int kReconfigureClose = 3; // Need to close and reopen sensor to change this setting
  static const int kReconfigureStop = 1; // Need to stop the stream before changing this setting
  static const int kReconfigureRunning = 0; // We can change this setting without stopping the stream

  void OpenCamera(UVCCameraConfig &new_config);
  void CloseCamera();

  // Accept a reconfigure request from a client
  void ReconfigureCallback(UVCCameraConfig &config, uint32_t level);
  enum uvc_frame_format GetVideoMode(std::string vmode);
  // Accept changes in values of automatically updated controls
  void AutoControlsCallback(enum uvc_status_class status_class,
                            int event,
                            int selector,
                            enum uvc_status_attribute status_attribute,
                            void *data, size_t data_len);
  static void AutoControlsCallbackAdapter(enum uvc_status_class status_class,
                                          int event,
                                          int selector,
                                          enum uvc_status_attribute status_attribute,
                                          void *data, size_t data_len,
                                          void *ptr);
  // Accept a new image frame from the camera
  void ImageCallback(uvc_frame_t *frame);
  static void ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);
  bool getUVCExposureCb(astra_camera::GetUVCExposureRequest& req, astra_camera::GetUVCExposureResponse& res);
  bool setUVCExposureCb(astra_camera::SetUVCExposureRequest& req, astra_camera::SetUVCExposureResponse& res);
  bool getUVCGainCb(astra_camera::GetUVCGainRequest& req, astra_camera::GetUVCGainResponse& res);
  bool setUVCGainCb(astra_camera::SetUVCGainRequest& req, astra_camera::SetUVCGainResponse& res);
  bool getUVCWhiteBalanceCb(astra_camera::GetUVCWhiteBalanceRequest& req, astra_camera::GetUVCWhiteBalanceResponse& res);
  bool setUVCWhiteBalanceCb(astra_camera::SetUVCWhiteBalanceRequest& req, astra_camera::SetUVCWhiteBalanceResponse& res);

  ros::NodeHandle nh_, priv_nh_;

  State state_;
  boost::recursive_mutex mutex_;

  uvc_context_t *ctx_;
  uvc_device_t *dev_;
  uvc_device_handle_t *devh_;
  uvc_frame_t *rgb_frame_;

  image_transport::ImageTransport it_;
  image_transport::CameraPublisher cam_pub_;

  dynamic_reconfigure::Server<UVCCameraConfig> config_server_;
  UVCCameraConfig config_;
  bool config_changed_;

  camera_info_manager::CameraInfoManager cinfo_manager_;
  bool param_init_;
  std::string ns;
  std::string ns_no_slash;

  ros::ServiceServer get_uvc_exposure_server;
  ros::ServiceServer set_uvc_exposure_server;
  ros::ServiceServer get_uvc_gain_server;
  ros::ServiceServer set_uvc_gain_server;
  ros::ServiceServer get_uvc_white_balance_server;
  ros::ServiceServer set_uvc_white_balance_server;

  ros::ServiceClient device_type_client;
  ros::ServiceClient camera_info_client;

  bool device_type_init_;
  bool camera_info_init_;
  std::string device_type_;
  sensor_msgs::CameraInfo camera_info_;
  int uvc_flip_;
  OB_DEVICE_NO device_type_no_;
  bool camera_info_valid_;
};

};
