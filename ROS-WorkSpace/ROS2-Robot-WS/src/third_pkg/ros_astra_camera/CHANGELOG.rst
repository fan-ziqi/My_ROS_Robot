^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package astra_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.3.0 (2019-08-08)
------------
* Add UVC support
* Add OPENNI2 include files under include folder
* Add more video modes
* Add useful services to control cameras
* Support Stereo S, Embedded S, and Stereo S U3
* Merge astra_launch package
* 56-orbbec-usb.rules now includes uvc support
* CMakeLists.txt changes accordingly
* Update README
* Contributors: Chi-Ju Wu, Nan Xu, dandedrick, coxep

0.2.2 (2018-03-22)
-----------
* Add launchfile info and reformat nicely
* Publish projector/camera_info (fixes disparity img)
* modify gcc  optimizate problem
* Contributors: Chan Jun Shern, Martin Günther, Mikael Arguedas, Tim

0.2.1 (2018-02-12)
-----------
* 1,patch for catkin_make -j1 in docker 2,patch for x64 use x86 in docker 3,arm/arm64 use no-filter library
* Contributors: Tim

0.2.0 (2018-01-25)
------------------
* add support for astra mini s
* Contributors: Tim Liu

0.1.5 (2016-05-27)
------------------
* add dependency on generated messages to avoid race condition at build time
* switching to libusb-1.0
* Contributors: Tully Foote

0.1.4 (2016-05-27)
------------------
* add libusb-dev as a build dependency
* Contributors: Tully Foote

0.1.3 (2016-05-26)
------------------
* Adding build dependency on libudev-dev
* Contributors: Tully Foote

0.1.2 (2016-05-26)
------------------
* add git as a dependency
* Contributors: Tully Foote

0.1.1 (2016-05-26)
------------------
* removing dependency which was internalilzed
* Contributors: Tully Foote

0.1.0 (2016-05-26)
------------------
* Initial release of ROS driver for Astra camera
* Contributors: Ernesto Corbellini, Len Zhong, Tim, Tully Foote, ob-tim-liu
