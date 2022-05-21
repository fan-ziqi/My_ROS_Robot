/*
 * Author: Kaven Yau (kavenyau@foxmail.com)
 */

#ifndef UVC_CAMERA_CONFIG_H_
#define UVC_CAMERA_CONFIG_H_

#include <string>

namespace ros2_astra_camera {
struct UVCCameraConfig {
  std::string vendor;
  std::string product;
  std::string serial;
  int index;
  int width;
  int height;
  std::string video_mode;
  double frame_rate;
  std::string timestamp_method;
  std::string frame_id;
  std::string camera_info_url;
  int scanning_mode;
  int auto_exposure;
  int auto_exposure_priority;
  double exposure_absolute;
  double iris_absolute;
  bool auto_focus;
  int focus_absolute;
  int pan_absolute;
  int tilt_absolute;
  int roll_absolute;
  bool privacy;
  int backlight_compensation;
  int brightness;
  int contrast;
  int gain;
  int power_line_frequency;
  bool auto_hue;
  double hue;
  int saturation;
  int sharpness;
  double gamma;
  bool auto_white_balance;
  int white_balance_temperature;
  int white_balance_BU;
  int white_balance_RV;
};
}  // namespace ros2_astra_camera

#endif  // UVC_CAMERA_CONFIG_H_