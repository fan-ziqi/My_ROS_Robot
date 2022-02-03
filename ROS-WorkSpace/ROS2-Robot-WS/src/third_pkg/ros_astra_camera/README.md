# astra_camera

A ROS driver for Orbbec 3D cameras.

## Install

This package supports ROS Kinetic and Melodic.

1. Install [ROS](http://wiki.ros.org/ROS/Installation).

2. Install dependences
    ```sh
    sudo apt install ros-$ROS_DISTRO-rgbd-launch ros-$ROS_DISTRO-libuvc ros-$ROS_DISTRO-libuvc-camera ros-$ROS_DISTRO-libuvc-ros
    ```

3. Create a [ROS Workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)(if you don't have one)
	 ```sh
    mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/
	catkin_make
	source devel/setup.bash
    ```
	
4. Pull the repository into your ROS workspace
    ```sh
    cd ~/catkin_ws/src
    git clone https://github.com/orbbec/ros_astra_camera
    ```

5. Create astra udev rule
    ```sh
    roscd astra_camera
    ./scripts/create_udev_rules
    ```

6. Go to catkin workspace and compile astra_camera
    ```sh
    cd ~/catkin_ws
    catkin_make --pkg astra_camera
    ```

### Filter Enable (To be deprecated: recommend to try master branch at first)

Astra driver provides normal and filtering methods. With filter driver, we can get more precise depth data but it would cost more computing resources. If the program will be executed on embedded systems, we suggest to use normal method. You can use `-DFILTER=ON / OFF` to change the method as below.

`catkin_make --pkg astra_camera -DFILTER=OFF`

## Run astra_camera

If you didn't add `source $YOUR_WORKSPACE/devel/setup.bash` to your `.bashrc`, remember to source it when open a new terminal :)

### Examples

#### Use Astra

`roslaunch astra_camera astra.launch`

#### Use Astra Stereo S (w/ UVC)

`roslaunch astra_camera stereo_s.launch`

You can use **rviz** or **image_view** to verify the outputs.

## Important Topics

* `*/image_raw`: depth/rgb/ir raw images
  * If showing IR image is required, it would be more visible to normalize it from 16bit to 8bit (0 to 255)
* `*/image_rect_raw`: images rectified by intrinsic/extrinsic parameters
* `*/camera_info`: camera intrinsic/extrinsic parameters
* `/camera/depth/points`: point cloud without color information
* `/camera/depth_registered/points`: xyzrgb point cloud (Currently, RGB-D regestration supports default resolution only)

## Useful Services

This package provides multiple [ros services](http://wiki.ros.org/Services) for users to get useful information and set up devices. To know more about using these services, please check [this tutorial](http://wiki.ros.org/rosservice).

* `/camera/get_device_type`: return a string containing astra device type
* `/camera/get_ir_exposure`: get exposure value of ir camera
* `/camera/get_ir_gain`: get gain value of ir camera
* `/camera/get_serial`: get serial number
* `/camera/get_uvc_exposure`: get exposure value of rgb camera
* `/camera/get_uvc_gain`: get gain value of rgb camera
* `/camera/get_uvc_white_balance`: get white balance value of rgb camera
* `/camera/reset_ir_exposure`: reset ir exposure to default value
* `/camera/reset_ir_gain`: reset ir gain to default value
* `/camera/set_ir_exposure`: set ir exposure to specific value
* `/camera/set_ir_gain`: set ir gain to specific value
* `/camera/set_laser`: turn on (**true**) or turn off (**false**) laser
* `/camera/set_ldp`: turn on (**true**) or turn off (**false**) ldp
* `/camera/set_uvc_exposure`: set uvc exposure. (set **0** indicating auto mode)
* `/camera/set_uvc_gain`: set uvc gain
* `/camera/set_uvc_white_balance`: set uvc white balance (set **0** indicating auto mode)
* `/camera/set_ir_flood`: turn on (**true**) or turn off (**false**) ir flood
* `/camera/switch_ir_camera`: while using stereo_s/stereo_s_u3, you can switch (left/right) ir camera

### Examples

After launching an astra camera, you can get ir exposure by the following command
1. ir exposure
    ```sh
    rosservice call /camera/get_ir_exposure
    ```
    Next, you can change this value in this way
    ```sh
    rosservice call /camera/set_ir_exposure "{exposure: 50}" # Press tab to autocomplete
    ```

2. turn on/off laser
    ```sh
    rosservice call /camera/set_laser "{enable: true}" # turn on
    rosservice call /camera/set_laser "{enable: false}" # turn off
    ```

3. switch ir
    ```sh
    rosservice call /camera/switch_ir_camera "camera: 'left'" # left
    rosservice call /camera/switch_ir_camera "camera: 'right'" # right
    ```

For the other services, the usage is same as the above example.

## Multiple Cameras

To launch multiple cameras, you could modify `multi_astra.launch` to match your setting. The important settings are `device_x_id`(serial number of your devices), `3d_sensor`(name of launch file), and `has_uvc_serial`(does your camera's uvc have serial number).

If you received **USB Buffer Error**, you could try to increase your USBFS buffer size by the following command.
```sh
echo 64 > /sys/module/usbcore/parameters/usbfs_memory_mb # or maybe 128
```

## License

Copyright 2019 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

*Other names and brands may be claimed as the property of others*
