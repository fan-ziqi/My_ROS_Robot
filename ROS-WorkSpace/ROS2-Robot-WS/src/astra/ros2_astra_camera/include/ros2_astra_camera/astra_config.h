/*
 * Author: Kaven Yau (kavenyau@foxmail.com)
 */

#ifndef ASTRA_CONFIG_H_
#define ASTRA_CONFIG_H_

namespace astra_camera {
struct AstraConfig {
  bool rgb_preferred;
  int ir_mode;
  int color_mode;
  int depth_mode;
  bool depth_registration;
  bool color_depth_synchronization;
  // bool auto_exposure;
  bool auto_white_balance;
  int data_skip;
  double ir_time_offset;
  double color_time_offset;
  double depth_time_offset;
  double depth_ir_offset_x;
  double depth_ir_offset_y;
  int z_offset_mm;
  double z_scaling;
  bool use_device_time;
  bool keep_alive;
};
}  // namespace astra_camera

#endif  // ASTRA_CONFIG_H_