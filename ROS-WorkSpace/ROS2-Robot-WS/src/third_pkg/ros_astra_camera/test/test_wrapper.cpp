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

#include "astra_camera/astra_device_manager.h"
#include "astra_camera/astra_device.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>

#include <iostream>

using namespace std;
using namespace astra_wrapper;

int ir_counter_ = 0;
int color_counter_ = 0;
int depth_counter_ = 0;

void IRCallback(sensor_msgs::msg::Image::SharedPtr image)
{
  ++ir_counter_;
}

void ColorCallback(sensor_msgs::msg::Image::SharedPtr image)
{
  ++color_counter_;
}

void DepthCallback(sensor_msgs::msg::Image::SharedPtr image)
{
  ++depth_counter_;
}

int main()
{
  AstraDeviceManager device_manager;

  std::cout << device_manager;

  boost::shared_ptr<std::vector<std::string> > device_uris = device_manager.getConnectedDeviceURIs();

  BOOST_FOREACH(const std::string& uri, *device_uris)
  {
    boost::shared_ptr<AstraDevice> device = device_manager.getDevice(uri);

    std::cout << *device;

    size_t width = 1280;
    size_t height = 1024;
    double framerate = 30;
    size_t dwidth = 640;
    size_t dheight = 400;
    double dframerate = 30;
    AstraVideoMode color_video_mode{width, height, framerate, astra_wrapper::PixelFormat::PIXEL_FORMAT_RGB888};
    device->setColorVideoMode(color_video_mode);

    AstraVideoMode ir_video_mode{width, height, framerate, astra_wrapper::PixelFormat::PIXEL_FORMAT_GRAY16};
    device->setIRVideoMode(color_video_mode);

    AstraVideoMode depth_video_mode{dwidth, dheight, dframerate, astra_wrapper::PixelFormat::PIXEL_FORMAT_DEPTH_1_MM};
    device->setDepthVideoMode(depth_video_mode);

    device->setIRFrameCallback([](sensor_msgs::msg::Image::SharedPtr image) { IRCallback(image); });
    device->setColorFrameCallback([](sensor_msgs::msg::Image::SharedPtr image) { ColorCallback(image); });
    device->setDepthFrameCallback([](sensor_msgs::msg::Image::SharedPtr image) { DepthCallback(image); });

    ir_counter_ = 0;
    color_counter_ = 0;
    depth_counter_ = 0;

    device->startColorStream();
    device->startDepthStream();

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    device->stopAllStreams();

    std::cout<<std::endl;

    std::cout<<"Number of called to IRCallback: "<< ir_counter_ << std::endl;
    std::cout<<"Number of called to ColorCallback: "<< color_counter_ << std::endl;
    std::cout<<"Number of called to DepthCallback: "<< depth_counter_ << std::endl;
  }


  return 0;
}
