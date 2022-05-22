/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2016, Orbbec Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Tim Liu (liuhua@orbbec.com)
 */

#include "astra_camera/astra_driver.h"
#include "astra_camera/astra_exception.h"
#include "astra_camera/astra_device_type.h"

#include <unistd.h>  
#include <stdlib.h>  
#include <stdio.h>  
#include <sys/shm.h>  
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

#define  MULTI_ASTRA 1
namespace astra_wrapper
{

AstraDriver::AstraDriver(ros::NodeHandle& n, ros::NodeHandle& pnh) :
    nh_(n),
    pnh_(pnh),
    device_manager_(AstraDeviceManager::getSingelton()),
    config_init_(false),
    data_skip_ir_counter_(0),
    data_skip_color_counter_(0),
    data_skip_depth_counter_ (0),
    ir_subscribers_(false),
    color_subscribers_(false),
    depth_subscribers_(false),
    depth_raw_subscribers_(false),
    uvc_flip_(0)
{

  genVideoModeTableMap();

  readConfigFromParameterServer();

#if MULTI_ASTRA
	int bootOrder, devnums;
  if (!pnh.getParam("bootorder", bootOrder))
  {
    bootOrder = 0;
  }
  
  if (!pnh.getParam("devnums", devnums))
  {
    devnums = 1;
  }

	if( devnums>1 )
	{
		int shmid;
		char *shm = NULL;
		char *tmp;

		if(  bootOrder==1 )
		{
			if( (shmid = shmget((key_t)0401, 1, 0666|IPC_CREAT)) == -1 )   
			{ 
				ROS_ERROR("Create Share Memory Error:%s", strerror(errno));
			}
			shm = (char *)shmat(shmid, 0, 0);  
			*shm = 1;
			initDevice();
			ROS_INFO("*********** device_id %s already open device************************ ", device_id_.c_str());
			*shm = 2;
		}
		else 	
		{	
			if( (shmid = shmget((key_t)0401, 1, 0666|IPC_CREAT)) == -1 )   
			{ 
			  	ROS_ERROR("Create Share Memory Error:%s", strerror(errno));
			}
			shm = (char *)shmat(shmid, 0, 0);
			while( *shm!=bootOrder)
			{
				boost::this_thread::sleep(boost::posix_time::milliseconds(10));
			}

			 initDevice();
			 ROS_INFO("*********** device_id %s already open device************************ ", device_id_.c_str());
			*shm = (bootOrder+1);
		}
		if(  bootOrder==devnums )
		{
			if(shmdt(shm) == -1)  
			{  
				ROS_ERROR("shmdt failed\n");  
			} 
			if(shmctl(shmid, IPC_RMID, 0) == -1)  
			{  
				ROS_ERROR("shmctl(IPC_RMID) failed\n");  
			}
		 }
		 else
		 {
		 	if(shmdt(shm) == -1)  
			{  
				ROS_ERROR("shmdt failed\n");  
			} 
		 }
	 }
	 else
	 {
	 	initDevice();
	 }
#else
  initDevice();

#endif
  // Initialize dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(pnh_));
  reconfigure_server_->setCallback(boost::bind(&AstraDriver::configCb, this, _1, _2));

  while (!config_init_)
  {
    ROS_DEBUG("Waiting for dynamic reconfigure configuration.");
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }
  ROS_DEBUG("Dynamic reconfigure configuration received.");

  advertiseROSTopics();
}

AstraDriver::~AstraDriver() {
  device_->stopAllStreams();
}

void AstraDriver::advertiseROSTopics()
{

  // Allow remapping namespaces rgb, ir, depth, depth_registered
  ros::NodeHandle color_nh(nh_, "rgb");
  image_transport::ImageTransport color_it(color_nh);
  ros::NodeHandle ir_nh(nh_, "ir");
  image_transport::ImageTransport ir_it(ir_nh);
  ros::NodeHandle depth_nh(nh_, "depth");
  image_transport::ImageTransport depth_it(depth_nh);
  ros::NodeHandle depth_raw_nh(nh_, "depth");
  image_transport::ImageTransport depth_raw_it(depth_raw_nh);
  ros::NodeHandle projector_nh(nh_, "projector");
  // Advertise all published topics

  // Prevent connection callbacks from executing until we've set all the publishers. Otherwise
  // connectCb() can fire while we're advertising (say) "depth/image_raw", but before we actually
  // assign to pub_depth_raw_. Then pub_depth_raw_.getNumSubscribers() returns 0, and we fail to start
  // the depth generator.
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  // Asus Xtion PRO does not have an RGB camera
  //ROS_WARN("-------------has color sensor is %d----------- ", device_->hasColorSensor());
  if (device_->hasColorSensor())
  {
    image_transport::SubscriberStatusCallback itssc = boost::bind(&AstraDriver::imageConnectCb, this);
    ros::SubscriberStatusCallback rssc = boost::bind(&AstraDriver::imageConnectCb, this);
    pub_color_ = color_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
  }

  if (device_->hasIRSensor())
  {
    image_transport::SubscriberStatusCallback itssc = boost::bind(&AstraDriver::imageConnectCb, this);
    ros::SubscriberStatusCallback rssc = boost::bind(&AstraDriver::imageConnectCb, this);
    pub_ir_ = ir_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
  }

  if (device_->hasDepthSensor())
  {
    image_transport::SubscriberStatusCallback itssc = boost::bind(&AstraDriver::depthConnectCb, this);
    ros::SubscriberStatusCallback rssc = boost::bind(&AstraDriver::depthConnectCb, this);
    pub_depth_raw_ = depth_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
    pub_depth_ = depth_raw_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
    pub_projector_info_ = projector_nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1, rssc, rssc);
  }

  ////////// CAMERA INFO MANAGER

  // Pixel offset between depth and IR images.
  // By default assume offset of (5,4) from 9x7 correlation window.
  // NOTE: These are now (temporarily?) dynamically reconfigurable, to allow tweaking.
  //param_nh.param("depth_ir_offset_x", depth_ir_offset_x_, 5.0);
  //param_nh.param("depth_ir_offset_y", depth_ir_offset_y_, 4.0);

  // The camera names are set to [rgb|depth]_[serial#], e.g. depth_B00367707227042B.
  // camera_info_manager substitutes this for ${NAME} in the URL.
  std::string serial_number = device_->getStringID();
  std::string color_name, ir_name;

  color_name = "rgb_"   + serial_number;
  ir_name  = "depth_" + serial_number;

  // Load the saved calibrations, if they exist
  color_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(color_nh, color_name, color_info_url_);
  ir_info_manager_  = boost::make_shared<camera_info_manager::CameraInfoManager>(ir_nh,  ir_name,  ir_info_url_);

  get_serial_server = nh_.advertiseService("get_serial", &AstraDriver::getSerialCb, this);
  get_device_type_server = nh_.advertiseService("get_device_type", &AstraDriver::getDeviceTypeCb, this);
  get_ir_gain_server = nh_.advertiseService("get_ir_gain", &AstraDriver::getIRGainCb, this);
  set_ir_gain_server = nh_.advertiseService("set_ir_gain", &AstraDriver::setIRGainCb, this);
  get_ir_exposure_server = nh_.advertiseService("get_ir_exposure", &AstraDriver::getIRExposureCb, this);
  set_ir_exposure_server = nh_.advertiseService("set_ir_exposure", &AstraDriver::setIRExposureCb, this);
  set_ir_flood_server = nh_.advertiseService("set_ir_flood", &AstraDriver::setIRFloodCb, this);
  set_laser_server = nh_.advertiseService("set_laser", &AstraDriver::setLaserCb, this);
  set_ldp_server = nh_.advertiseService("set_ldp", &AstraDriver::setLDPCb, this);
  reset_ir_gain_server = nh_.advertiseService("reset_ir_gain", &AstraDriver::resetIRGainCb, this);
  reset_ir_exposure_server = nh_.advertiseService("reset_ir_exposure", &AstraDriver::resetIRExposureCb, this);
  get_camera_info = nh_.advertiseService("get_camera_info", &AstraDriver::getCameraInfoCb, this);
  if (device_->getDeviceTypeNo() == OB_STEREO_S_NO || device_->getDeviceTypeNo() == OB_STEREO_S_U3_NO)
  {
    switch_ir_camera = nh_.advertiseService("switch_ir_camera", &AstraDriver::switchIRCameraCb, this);
  }
}

bool AstraDriver::getSerialCb(astra_camera::GetSerialRequest& req, astra_camera::GetSerialResponse& res)
{
  res.serial = device_manager_->getSerial(device_->getUri());
  return true;
}

bool AstraDriver::getDeviceTypeCb(astra_camera::GetDeviceTypeRequest& req, astra_camera::GetDeviceTypeResponse& res)
{
  res.device_type = std::string(device_->getDeviceType());
  return true;
}

bool AstraDriver::getIRGainCb(astra_camera::GetIRGainRequest& req, astra_camera::GetIRGainResponse& res)
{
  res.gain = device_->getIRGain();
  return true;
}

bool AstraDriver::setIRGainCb(astra_camera::SetIRGainRequest& req, astra_camera::SetIRGainResponse& res)
{
  device_->setIRGain(req.gain);
  return true;
}

bool AstraDriver::getIRExposureCb(astra_camera::GetIRExposureRequest& req, astra_camera::GetIRExposureResponse& res)
{
  res.exposure = device_->getIRExposure();
  return true;
}

bool AstraDriver::setIRExposureCb(astra_camera::SetIRExposureRequest& req, astra_camera::SetIRExposureResponse& res)
{
  device_->setIRExposure(req.exposure);
  return true;
}

bool AstraDriver::setLaserCb(astra_camera::SetLaserRequest& req, astra_camera::SetLaserResponse& res)
{
  device_->setLaser(req.enable);
  return true;
}

bool AstraDriver::setLDPCb(astra_camera::SetLDPRequest& req, astra_camera::SetLDPResponse& res)
{
  device_->setLDP(req.enable);
  return true;
}

bool AstraDriver::resetIRGainCb(astra_camera::ResetIRGainRequest& req, astra_camera::ResetIRGainResponse& res)
{
  device_->setIRGain(0x8);
  return true;
}

bool AstraDriver::resetIRExposureCb(astra_camera::ResetIRExposureRequest& req, astra_camera::ResetIRExposureResponse& res)
{
  device_->setIRExposure(0x419);
  return true;
}

bool AstraDriver::getCameraInfoCb(astra_camera::GetCameraInfoRequest& req, astra_camera::GetCameraInfoResponse& res)
{
  res.info = convertAstraCameraInfo(device_->getCameraParams(), ros::Time::now());
  return true;
}

bool AstraDriver::setIRFloodCb(astra_camera::SetIRFloodRequest& req, astra_camera::SetIRFloodResponse& res)
{
  device_->setIRFlood(req.enable);
  return true;
}

bool AstraDriver::switchIRCameraCb(astra_camera::SwitchIRCameraRequest& req, astra_camera::SwitchIRCameraResponse& res)
{
  if (req.camera == "left")
    device_->switchIRCamera(0);
  else if (req.camera == "right")
    device_->switchIRCamera(1);
  else
    ROS_ERROR("Only support left/right");
  return true;
}

void AstraDriver::configCb(Config &config, uint32_t level)
{
  if (device_->getDeviceTypeNo() == OB_STEREO_S_NO)
  {
    if (config.depth_mode != 13 && config.depth_mode != 14)
    {
      config.depth_mode = 13;
    }
    if (config.ir_mode != 13 && config.ir_mode != 14 && config.ir_mode != 16)
    {
      config.ir_mode = 13;
    }
    device_->switchIRCamera(0);
  }
  else if (device_->getDeviceTypeNo() == OB_EMBEDDED_S_NO)
  {
    if (config.depth_mode != 13 && config.depth_mode != 17)
    {
      config.depth_mode = 13;
    }
    if (config.ir_mode != 13 && config.ir_mode != 17)
    {
      config.ir_mode = 13;
    }
    uvc_flip_ = 1;
  }
  else if (device_->getDeviceTypeNo() == OB_STEREO_S_U3_NO)
  {
    if (config.depth_mode != 13 && config.depth_mode != 14)
    {
      config.depth_mode = 13;
    }
    if (config.ir_mode != 13 && config.ir_mode != 14)
    {
      config.ir_mode = 13;
    }
    device_->switchIRCamera(0);
  }
  bool stream_reset = false;

  rgb_preferred_ = config.rgb_preferred;

  if (config_init_ && old_config_.rgb_preferred != config.rgb_preferred)
    imageConnectCb();

  depth_ir_offset_x_ = config.depth_ir_offset_x;
  depth_ir_offset_y_ = config.depth_ir_offset_y;
  z_offset_mm_ = config.z_offset_mm;
  z_scaling_ = config.z_scaling;

  ir_time_offset_ = ros::Duration(config.ir_time_offset);
  color_time_offset_ = ros::Duration(config.color_time_offset);
  depth_time_offset_ = ros::Duration(config.depth_time_offset);

  if (lookupVideoModeFromDynConfig(config.ir_mode, ir_video_mode_)<0)
  {
    ROS_ERROR("Undefined IR video mode received from dynamic reconfigure");
    exit(-1);
  }

  if (lookupVideoModeFromDynConfig(config.color_mode, color_video_mode_)<0)
  {
    ROS_ERROR("Undefined color video mode received from dynamic reconfigure");
    exit(-1);
  }

  if (lookupVideoModeFromDynConfig(config.depth_mode, depth_video_mode_)<0)
  {
    ROS_ERROR("Undefined depth video mode received from dynamic reconfigure");
    exit(-1);
  }

  // assign pixel format

  ir_video_mode_.pixel_format_ = PIXEL_FORMAT_GRAY16;
  color_video_mode_.pixel_format_ = PIXEL_FORMAT_RGB888;
  depth_video_mode_.pixel_format_ = PIXEL_FORMAT_DEPTH_1_MM;

  color_depth_synchronization_ = config.color_depth_synchronization;
  depth_registration_ = config.depth_registration;

  auto_exposure_ = config.auto_exposure;
  auto_white_balance_ = config.auto_white_balance;

  use_device_time_ = config.use_device_time;

  data_skip_ = config.data_skip+1;

  applyConfigToOpenNIDevice();

  config_init_ = true;

  old_config_ = config;
}

void AstraDriver::setIRVideoMode(const AstraVideoMode& ir_video_mode)
{
  if (device_->isIRVideoModeSupported(ir_video_mode))
  {
    if (ir_video_mode != device_->getIRVideoMode())
    {
      device_->setIRVideoMode(ir_video_mode);
    }

  }
  else
  {
    ROS_ERROR_STREAM("Unsupported IR video mode - " << ir_video_mode);
  }
}
void AstraDriver::setColorVideoMode(const AstraVideoMode& color_video_mode)
{
  if (device_->isColorVideoModeSupported(color_video_mode))
  {
    if (color_video_mode != device_->getColorVideoMode())
    {
      device_->setColorVideoMode(color_video_mode);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported color video mode - " << color_video_mode);
  }
}
void AstraDriver::setDepthVideoMode(const AstraVideoMode& depth_video_mode)
{
  if (device_->isDepthVideoModeSupported(depth_video_mode))
  {
    if (depth_video_mode != device_->getDepthVideoMode())
    {
      device_->setDepthVideoMode(depth_video_mode);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported depth video mode - " << depth_video_mode);
  }
}

void AstraDriver::applyConfigToOpenNIDevice()
{

  data_skip_ir_counter_ = 0;
  data_skip_color_counter_= 0;
  data_skip_depth_counter_ = 0;

  setIRVideoMode(ir_video_mode_);
  if (device_->hasColorSensor())
  {
  	setColorVideoMode(color_video_mode_);
  }
  setDepthVideoMode(depth_video_mode_);

  if (device_->isImageRegistrationModeSupported())
  {
    try
    {
      if (!config_init_ || (old_config_.depth_registration != depth_registration_))
        device_->setImageRegistrationMode(depth_registration_);
    }
    catch (const AstraException& exception)
    {
      ROS_ERROR("Could not set image registration. Reason: %s", exception.what());
    }
  }

  try
  {
    if (!config_init_ || (old_config_.color_depth_synchronization != color_depth_synchronization_))
      device_->setDepthColorSync(color_depth_synchronization_);
  }
  catch (const AstraException& exception)
  {
    ROS_ERROR("Could not set color depth synchronization. Reason: %s", exception.what());
  }

  try
  {
    if (!config_init_ || (old_config_.auto_exposure != auto_exposure_))
      device_->setAutoExposure(auto_exposure_);
  }
  catch (const AstraException& exception)
  {
    ROS_ERROR("Could not set auto exposure. Reason: %s", exception.what());
  }

  try
  {
    if (!config_init_ || (old_config_.auto_white_balance != auto_white_balance_))
      device_->setAutoWhiteBalance(auto_white_balance_);
  }
  catch (const AstraException& exception)
  {
    ROS_ERROR("Could not set auto white balance. Reason: %s", exception.what());
  }

  device_->setUseDeviceTimer(use_device_time_);

}

void AstraDriver::imageConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  bool ir_started = device_->isIRStreamStarted();
  bool color_started = device_->isColorStreamStarted();

  ir_subscribers_ = pub_ir_.getNumSubscribers() > 0;
  color_subscribers_ = pub_color_.getNumSubscribers() > 0;

  if (color_subscribers_ && (!ir_subscribers_ || rgb_preferred_))
  {
    if (ir_subscribers_)
      ROS_ERROR("Cannot stream RGB and IR at the same time. Streaming RGB only.");

    if (ir_started)
    {
      ROS_INFO("Stopping IR stream.");
      device_->stopIRStream();
    }

    if (!color_started)
    {
      device_->setColorFrameCallback(boost::bind(&AstraDriver::newColorFrameCallback, this, _1));

      ROS_INFO("Starting color stream.");
      device_->startColorStream();
    }
  }
  else if (ir_subscribers_ && (!color_subscribers_ || !rgb_preferred_))
  {

    if (color_subscribers_)
      ROS_ERROR("Cannot stream RGB and IR at the same time. Streaming IR only.");

    if (color_started)
    {
      ROS_INFO("Stopping color stream.");
      device_->stopColorStream();
    }

    if (!ir_started)
    {
      device_->setIRFrameCallback(boost::bind(&AstraDriver::newIRFrameCallback, this, _1));

      ROS_INFO("Starting IR stream.");
      device_->startIRStream();
    }
  }
  else
  {
    if (color_started)
    {
      ROS_INFO("Stopping color stream.");
      device_->stopColorStream();
    }
    if (ir_started)
    {
      ROS_INFO("Stopping IR stream.");
      device_->stopIRStream();
    }
  }
}

void AstraDriver::depthConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  depth_subscribers_ = pub_depth_.getNumSubscribers() > 0;
  depth_raw_subscribers_ = pub_depth_raw_.getNumSubscribers() > 0;
  projector_info_subscribers_ = pub_projector_info_.getNumSubscribers() > 0;

  bool need_depth = depth_subscribers_ || depth_raw_subscribers_;

  if (need_depth && !device_->isDepthStreamStarted())
  {
    device_->setDepthFrameCallback(boost::bind(&AstraDriver::newDepthFrameCallback, this, _1));

    ROS_INFO("Starting depth stream.");
    device_->startDepthStream();
  }
  else if (!need_depth && device_->isDepthStreamStarted())
  {
    ROS_INFO("Stopping depth stream.");
    device_->stopDepthStream();
  }
}

void AstraDriver::newIRFrameCallback(sensor_msgs::ImagePtr image)
{
  if ((++data_skip_ir_counter_)%data_skip_==0)
  {
    data_skip_ir_counter_ = 0;

    if (ir_subscribers_)
    {
      image->header.frame_id = ir_frame_id_;
      image->header.stamp = image->header.stamp + ir_time_offset_;

      pub_ir_.publish(image, getIRCameraInfo(image->width, image->height, image->header.stamp));
    }
  }
}

void AstraDriver::newColorFrameCallback(sensor_msgs::ImagePtr image)
{
  if ((++data_skip_color_counter_)%data_skip_==0)
  {
    data_skip_color_counter_ = 0;

    if (color_subscribers_)
    {
      image->header.frame_id = color_frame_id_;
      image->header.stamp = image->header.stamp + color_time_offset_;

      pub_color_.publish(image, getColorCameraInfo(image->width, image->height, image->header.stamp));
    }
  }
}

void AstraDriver::newDepthFrameCallback(sensor_msgs::ImagePtr image)
{
  if ((++data_skip_depth_counter_)%data_skip_==0)
  {

    data_skip_depth_counter_ = 0;

    if (depth_raw_subscribers_||depth_subscribers_||projector_info_subscribers_)
    {
      image->header.stamp = image->header.stamp + depth_time_offset_;

      if (z_offset_mm_ != 0)
      {
        uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
        for (unsigned int i = 0; i < image->width * image->height; ++i)
          if (data[i] != 0)
                data[i] += z_offset_mm_;
      }

      if (fabs(z_scaling_ - 1.0) > 1e-6)
      {
        uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
        for (unsigned int i = 0; i < image->width * image->height; ++i)
          if (data[i] != 0)
                data[i] = static_cast<uint16_t>(data[i] * z_scaling_);
      }

      sensor_msgs::CameraInfoPtr cam_info;

      if (depth_registration_)
      {
        image->header.frame_id = color_frame_id_;
      } else
      {
        image->header.frame_id = depth_frame_id_;
      }
      cam_info = getDepthCameraInfo(image->width, image->height, image->header.stamp);

      if (depth_raw_subscribers_)
      {
        pub_depth_raw_.publish(image, cam_info);
      }

      if (depth_subscribers_ )
      {
        sensor_msgs::ImageConstPtr floating_point_image = rawToFloatingPointConversion(image);
        pub_depth_.publish(floating_point_image, cam_info);
      }

      // Projector "info" probably only useful for working with disparity images
      if (projector_info_subscribers_)
      {
        pub_projector_info_.publish(getProjectorCameraInfo(image->width, image->height, image->header.stamp));
      }
    }
  }
}

sensor_msgs::CameraInfo AstraDriver::convertAstraCameraInfo(OBCameraParams p, ros::Time time) const
{
  sensor_msgs::CameraInfo info;
  // info.width = width;
  // info.height = height;
  info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  info.D.resize(5, 0.0);
  info.D[0] = p.r_k[0];
  info.D[1] = p.r_k[1];
  info.D[2] = p.r_k[3];
  info.D[3] = p.r_k[4];
  info.D[4] = p.r_k[2];

  info.K.assign(0.0);
  info.K[0] = p.r_intr_p[0];
  info.K[2] = p.r_intr_p[2];
  info.K[4] = p.r_intr_p[1];
  info.K[5] = p.r_intr_p[3];
  info.K[8] = 1.0;

  info.R.assign(0.0);
  for (int i = 0; i < 9; i++)
  {
    info.R[i] = p.r2l_r[i];
  }

  info.P.assign(0.0);
  info.P[0] = info.K[0];
  info.P[2] = info.K[2];
  info.P[3] = p.r2l_t[0];
  info.P[5] = info.K[4];
  info.P[6] = info.K[5];
  info.P[7] = p.r2l_t[1];
  info.P[10] = 1.0;
  info.P[11] = p.r2l_t[2];
  // Fill in header
  info.header.stamp    = time;
  info.header.frame_id = color_frame_id_;
  return info;
}

// Methods to get calibration parameters for the various cameras
sensor_msgs::CameraInfoPtr AstraDriver::getDefaultCameraInfo(int width, int height, double f) const
{
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

  info->width  = width;
  info->height = height;

  // No distortion
  info->D.resize(5, 0.0);
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // Simple camera matrix: square pixels (fx = fy), principal point at center
  info->K.assign(0.0);
  info->K[0] = info->K[4] = f;
  info->K[2] = (width / 2) - 0.5;
  // Aspect ratio for the camera center on Astra (and other devices?) is 4/3
  // This formula keeps the principal point the same in VGA and SXGA modes
  info->K[5] = (width * (3./8.)) - 0.5;
  info->K[8] = 1.0;

  // No separate rectified image plane, so R = I
  info->R.assign(0.0);
  info->R[0] = info->R[4] = info->R[8] = 1.0;

  // Then P=K(I|0) = (K|0)
  info->P.assign(0.0);
  info->P[0]  = info->P[5] = f; // fx, fy
  info->P[2]  = info->K[2];     // cx
  info->P[6]  = info->K[5];     // cy
  info->P[10] = 1.0;

  return info;
}

/// @todo Use binning/ROI properly in publishing camera infos
sensor_msgs::CameraInfoPtr AstraDriver::getColorCameraInfo(int width, int height, ros::Time time) const
{
  sensor_msgs::CameraInfoPtr info;

  if (color_info_manager_->isCalibrated())
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>(color_info_manager_->getCameraInfo());
    if ( info->width != width )
    {
      // Use uncalibrated values
      ROS_WARN_ONCE("Image resolution doesn't match calibration of the RGB camera. Using default parameters.");
      info = getDefaultCameraInfo(width, height, device_->getColorFocalLength(height));
    }
  }
  else
  {
    // If uncalibrated, fill in default values
    if (device_->isCameraParamsValid())
    {
      sensor_msgs::CameraInfo cinfo = convertAstraCameraInfo(device_->getCameraParams(), time);
      info = boost::make_shared<sensor_msgs::CameraInfo>(ir_info_manager_->getCameraInfo());
      info->D.resize(5, 0.0);
      info->K.assign(0.0);
      info->R.assign(0.0);
      info->P.assign(0.0);
      info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      info->width = width;
      info->height = height;

      for (int i = 0; i < 9; i++)
      {
        info->K[i] = cinfo.K[i];
        info->R[i] = cinfo.R[i];
      }

      for (int i = 0; i < 12; i++)
      {
        info->P[i] = cinfo.P[i];
      }
    }
    else
    {
      info = getDefaultCameraInfo(width, height, device_->getColorFocalLength(height));
    }
  }

  // Fill in header
  info->header.stamp    = time;
  info->header.frame_id = color_frame_id_;

  return info;
}


sensor_msgs::CameraInfoPtr AstraDriver::getIRCameraInfo(int width, int height, ros::Time time) const
{
  sensor_msgs::CameraInfoPtr info;

  if (ir_info_manager_->isCalibrated())
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>(ir_info_manager_->getCameraInfo());
    if ( info->width != width )
    {
      // Use uncalibrated values
      ROS_WARN_ONCE("Image resolution doesn't match calibration of the IR camera. Using default parameters.");
      info = getDefaultCameraInfo(width, height, device_->getIRFocalLength(height));
    }
  }
  else
  {
    // If uncalibrated, fill in default values
    info = getDefaultCameraInfo(width, height, device_->getDepthFocalLength(height));

    if (device_->isCameraParamsValid())
    {
      OBCameraParams p = device_->getCameraParams();
      info->D.resize(5, 0.0);
      // info->D[0] = p.l_k[0];
      // info->D[1] = p.l_k[1];
      // info->D[2] = p.l_k[3];
      // info->D[3] = p.l_k[4];
      // info->D[4] = p.l_k[2];

      info->K.assign(0.0);
      info->K[0] = p.l_intr_p[0];
      info->K[2] = p.l_intr_p[2];
      info->K[4] = p.l_intr_p[1];
      info->K[5] = p.l_intr_p[3];
      info->K[8] = 1.0;

      info->R.assign(0.0);
      info->R[0] = 1;
      info->R[4] = 1;
      info->R[8] = 1;

      info->P.assign(0.0);
      info->P[0] = info->K[0];
      info->P[2] = info->K[2];
      info->P[5] = info->K[4];
      info->P[6] = info->K[5];
      info->P[10] = 1.0;
    }
  }

  // Fill in header
  info->header.stamp    = time;
  info->header.frame_id = depth_frame_id_;

  return info;
}

sensor_msgs::CameraInfoPtr AstraDriver::getDepthCameraInfo(int width, int height, ros::Time time) const
{
  // The depth image has essentially the same intrinsics as the IR image, BUT the
  // principal point is offset by half the size of the hardware correlation window
  // (probably 9x9 or 9x7 in 640x480 mode). See http://www.ros.org/wiki/kinect_calibration/technical

  double scaling = (double)width / 640;

  sensor_msgs::CameraInfoPtr info = getIRCameraInfo(width, height, time);
  info->K[2] -= depth_ir_offset_x_*scaling; // cx
  info->K[5] -= depth_ir_offset_y_*scaling; // cy
  info->P[2] -= depth_ir_offset_x_*scaling; // cx
  info->P[6] -= depth_ir_offset_y_*scaling; // cy

  /// @todo Could put this in projector frame so as to encode the baseline in P[3]
  return info;
}

sensor_msgs::CameraInfoPtr AstraDriver::getProjectorCameraInfo(int width, int height, ros::Time time) const
{
  // The projector info is simply the depth info with the baseline encoded in the P matrix.
  // It's only purpose is to be the "right" camera info to the depth camera's "left" for
  // processing disparity images.
  sensor_msgs::CameraInfoPtr info = getDepthCameraInfo(width, height, time);
  // Tx = -baseline * fx
  info->P[3] = -device_->getBaseline() * info->P[0];
  return info;
}

void AstraDriver::readConfigFromParameterServer()
{
  if (!pnh_.getParam("device_id", device_id_))
  {
    ROS_WARN ("~device_id is not set! Using first device.");
    device_id_ = "#1";
  }

  // Camera TF frames
  pnh_.param("ir_frame_id", ir_frame_id_, std::string("/openni_ir_optical_frame"));
  pnh_.param("rgb_frame_id", color_frame_id_, std::string("/openni_rgb_optical_frame"));
  pnh_.param("depth_frame_id", depth_frame_id_, std::string("/openni_depth_optical_frame"));

  ROS_DEBUG("ir_frame_id = '%s' ", ir_frame_id_.c_str());
  ROS_DEBUG("rgb_frame_id = '%s' ", color_frame_id_.c_str());
  ROS_DEBUG("depth_frame_id = '%s' ", depth_frame_id_.c_str());

  pnh_.param("rgb_camera_info_url", color_info_url_, std::string());
  pnh_.param("depth_camera_info_url", ir_info_url_, std::string());

}

std::string AstraDriver::resolveDeviceURI(const std::string& device_id)
{
  // retrieve available device URIs, they look like this: "1d27/0601@1/5"
  // which is <vendor ID>/<product ID>@<bus number>/<device number>
  boost::shared_ptr<std::vector<std::string> > available_device_URIs =
    device_manager_->getConnectedDeviceURIs();

  //for tes
  #if 0
   for (size_t i = 0; i < available_device_URIs->size(); ++i)
   {
       std::string s = (*available_device_URIs)[i];
  	ROS_WARN("------------id %d, available_device_uri is %s-----------", i, s.c_str());
   }
   #endif
  //end
  // look for '#<number>' format
  if (device_id.size() > 1 && device_id[0] == '#')
  {
    std::istringstream device_number_str(device_id.substr(1));
    int device_number;
    device_number_str >> device_number;
    int device_index = device_number - 1; // #1 refers to first device
    if (device_index >= available_device_URIs->size() || device_index < 0)
    {
      THROW_OPENNI_EXCEPTION(
          "Invalid device number %i, there are %zu devices connected.",
          device_number, available_device_URIs->size());
    }
    else
    {
      return available_device_URIs->at(device_index);
    }
  }
  // look for '<bus>@<number>' format
  //   <bus>    is usb bus id, typically start at 1
  //   <number> is the device number, for consistency with astra_camera, these start at 1
  //               although 0 specifies "any device on this bus"
  else if (device_id.size() > 1 && device_id.find('@') != std::string::npos && device_id.find('/') == std::string::npos)
  {
    // get index of @ character
    size_t index = device_id.find('@');
    if (index <= 0)
    {
      THROW_OPENNI_EXCEPTION(
        "%s is not a valid device URI, you must give the bus number before the @.",
        device_id.c_str());
    }
    if (index >= device_id.size() - 1)
    {
      THROW_OPENNI_EXCEPTION(
        "%s is not a valid device URI, you must give a number after the @, specify 0 for first device",
        device_id.c_str());
    }

    // pull out device number on bus
    std::istringstream device_number_str(device_id.substr(index+1));
    int device_number;
    device_number_str >> device_number;

    // reorder to @<bus>
    std::string bus = device_id.substr(0, index);
    bus.insert(0, "@");

    for (size_t i = 0; i < available_device_URIs->size(); ++i)
    {
      std::string s = (*available_device_URIs)[i];
      if (s.find(bus) != std::string::npos)
      {
        // this matches our bus, check device number
        --device_number;
        if (device_number <= 0)
          return s;
      }
    }

    THROW_OPENNI_EXCEPTION("Device not found %s", device_id.c_str());
  }
  else
  {
    // check if the device id given matches a serial number of a connected device
    for(std::vector<std::string>::const_iterator it = available_device_URIs->begin();
        it != available_device_URIs->end(); ++it)
    {
	#if 0
      	try 
	{
        	std::string serial = device_manager_->getSerial(*it);
        	if (serial.size() > 0 && device_id == serial)
          		return *it;
	}
    	#else
	try 
	{
         	std::set<std::string>::iterator iter;
        	if((iter = alreadyOpen.find(*it)) == alreadyOpen.end())
        	{
              		// ROS_WARN("------------seraial num it is  %s, device_id is %s -----------", (*it).c_str(), device_id_.c_str());
        		std::string serial = device_manager_->getSerial(*it);
        	 	if (serial.size() > 0 && device_id == serial)
        		{
          			alreadyOpen.insert(*it);
          			return *it;
         		}
        	}
      	}
	#endif
      	catch (const AstraException& exception)
      	{
        	ROS_WARN("Could not query serial number of device \"%s\":", exception.what());
      	}
    }

    // everything else is treated as part of the device_URI
    bool match_found = false;
    std::string matched_uri;
    for (size_t i = 0; i < available_device_URIs->size(); ++i)
    {
      std::string s = (*available_device_URIs)[i];
      if (s.find(device_id) != std::string::npos)
      {
        if (!match_found)
        {
          matched_uri = s;
          match_found = true;
        }
        else
        {
          // more than one match
          THROW_OPENNI_EXCEPTION("Two devices match the given device id '%s': %s and %s.", device_id.c_str(), matched_uri.c_str(), s.c_str());
        }
      }
    }
    return matched_uri;
  }

  return "INVALID";
}

void AstraDriver::initDevice()
{
  while (ros::ok() && !device_)
  {
    try
    {
      std::string device_URI = resolveDeviceURI(device_id_);
      #if 0
      if( device_URI == "" ) 
      {
      	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
      	continue;
      }
      #endif
      device_ = device_manager_->getDevice(device_URI);
    }
    catch (const AstraException& exception)
    {
      if (!device_)
      {
        ROS_INFO("No matching device found.... waiting for devices. Reason: %s", exception.what());
        boost::this_thread::sleep(boost::posix_time::seconds(3));
        continue;
      }
      else
      {
        ROS_ERROR("Could not retrieve device. Reason: %s", exception.what());
        exit(-1);
      }
    }
  }

  while (ros::ok() && !device_->isValid())
  {
    ROS_DEBUG("Waiting for device initialization..");
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }

}

void AstraDriver::genVideoModeTableMap()
{
  /*
   * #  Video modes defined by dynamic reconfigure:
output_mode_enum = gen.enum([  gen.const(  "SXGA_30Hz", int_t, 1,  "1280x1024@30Hz"),
                               gen.const(  "SXGA_15Hz", int_t, 2,  "1280x1024@15Hz"),
                               gen.const(   "XGA_30Hz", int_t, 3,  "1280x720@30Hz"),
                               gen.const(   "XGA_15Hz", int_t, 4,  "1280x720@15Hz"),
                               gen.const(   "VGA_30Hz", int_t, 5,  "640x480@30Hz"),
                               gen.const(   "VGA_25Hz", int_t, 6,  "640x480@25Hz"),
                               gen.const(  "QVGA_25Hz", int_t, 7,  "320x240@25Hz"),
                               gen.const(  "QVGA_30Hz", int_t, 8,  "320x240@30Hz"),
                               gen.const(  "QVGA_60Hz", int_t, 9,  "320x240@60Hz"),
                               gen.const( "QQVGA_25Hz", int_t, 10, "160x120@25Hz"),
                               gen.const( "QQVGA_30Hz", int_t, 11, "160x120@30Hz"),
                               gen.const( "QQVGA_60Hz", int_t, 12, "160x120@60Hz"),
                               gen.const("640400_30Hz", int_t, 13, "640x400@30Hz"),
                               gen.const("320200_30Hz", int_t, 14, "320x200@30Hz"),
                               gen.const("1280800_7Hz", int_t, 15, "1280x800@7Hz"),
                               gen.const("1280800_30Hz", int_t, 16, "1280x800@30Hz"),
                               gen.const("640400_60Hz", int_t, 17, "640x400@60Hz")],
                               "output mode")
  */

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

  // XGA_30Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 720;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[3] = video_mode;

  // XGA_15Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 720;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[4] = video_mode;

  // VGA_30Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[5] = video_mode;

  // VGA_25Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_[6] = video_mode;

  // QVGA_25Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_[7] = video_mode;

  // QVGA_30Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[8] = video_mode;

  // QVGA_60Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[9] = video_mode;

  // QQVGA_25Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_[10] = video_mode;

  // QQVGA_30Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[11] = video_mode;

  // QQVGA_60Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[12] = video_mode;

  // 640*400_30Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 400;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[13] = video_mode;

  // 320*200_30Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 200;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[14] = video_mode;

  // 1280*800_7Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 800;
  video_mode.frame_rate_ = 7;

  video_modes_lookup_[15] = video_mode;

  // 1280*800_30Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 800;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[16] = video_mode;

  // 640*400_60Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 400;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[17] = video_mode;
}

int AstraDriver::lookupVideoModeFromDynConfig(int mode_nr, AstraVideoMode& video_mode)
{
  int ret = -1;

  std::map<int, AstraVideoMode>::const_iterator it;

  it = video_modes_lookup_.find(mode_nr);

  if (it!=video_modes_lookup_.end())
  {
    video_mode = it->second;
    ret = 0;
  }

  return ret;
}

sensor_msgs::ImageConstPtr AstraDriver::rawToFloatingPointConversion(sensor_msgs::ImageConstPtr raw_image)
{
  static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

  sensor_msgs::ImagePtr new_image = boost::make_shared<sensor_msgs::Image>();

  new_image->header = raw_image->header;
  new_image->width = raw_image->width;
  new_image->height = raw_image->height;
  new_image->is_bigendian = 0;
  new_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  new_image->step = sizeof(float)*raw_image->width;

  std::size_t data_size = new_image->width*new_image->height;
  new_image->data.resize(data_size*sizeof(float));

  const unsigned short* in_ptr = reinterpret_cast<const unsigned short*>(&raw_image->data[0]);
  float* out_ptr = reinterpret_cast<float*>(&new_image->data[0]);

  for (std::size_t i = 0; i<data_size; ++i, ++in_ptr, ++out_ptr)
  {
    if (*in_ptr==0 || *in_ptr==0x7FF)
    {
      *out_ptr = bad_point;
    } else
    {
      *out_ptr = static_cast<float>(*in_ptr)/1000.0f;
    }
  }

  return new_image;
}

}
