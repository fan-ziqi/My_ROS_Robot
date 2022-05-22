/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#ifndef ONIENUMS_H
#define ONIENUMS_H

namespace openni
{

/** Possible failure values */
typedef enum
{
	STATUS_OK = 0,
	STATUS_ERROR = 1,
	STATUS_NOT_IMPLEMENTED = 2,
	STATUS_NOT_SUPPORTED = 3,
	STATUS_BAD_PARAMETER = 4,
	STATUS_OUT_OF_FLOW = 5,
	STATUS_NO_DEVICE = 6,
	STATUS_NOT_WRITE_PUBLIC_KEY = 7,
	STATUS_PUBLIC_KEY_MD5_VERIFY_FAILED = 8,
	STATUS_NOT_WRITE_MD5 = 9,
	STATUS_RSKEY_VERIFY_FAILED =10,

	//New add
	STATUS_DEVICE_IS_ALREADY_OPENED = 0x1001,                   //Device is already opened
	STATUS_USB_DRIVER_NOT_FOUND = 0x1002,                       //USB driver not found,for windows!
	STATUS_USB_DEVICE_NOT_FOUND = 0x1003,                       //USB device not found
	STATUS_USB_GET_DRIVER_VERSION = 0x1004,                     //Failed to get the USB driver version,for windows!
	STATUS_USB_GET_SPEED_FAILED = 0x1005,                       //Failed to get the device speed!
	STATUS_USB_SET_INTERFACE_FAILED = 0x1006,                   //Failed to set USB interface!
	STATUS_USB_DEVICE_OPEN_FAILED = 0x1007,						//pid,vid,bus id is zero,for linux
	STATUS_USB_ENUMERATE_FAILED = 0x1008,						//USB enum fail,for linux
	STATUS_USB_SET_CONFIG_FAILED = 0x1009,						//usb set config fail ,for linux
	STATUS_USB_INIT_FAILED = 0x1010,							//libusb open fail
	STATUS_LIBUSB_ERROR_NO_MEM = 0x1011,						//Insufficient memory
	STATUS_LIBUSB_ERROR_ACCESS = 0x1012,						//Access denied (insufficient permissions)
	STATUS_LIBUSB_ERROR_NO_DEVICE = 0x1013,						//No such device (it may have been disconnected)
	STATUS_LIBUSB_ERROR_IO = 0x1014,							//Input/output error
	STATUS_LIBUSB_ERROR_NOT_FOUND = 0x1015,                     //Entity not found
	STATUS_LIBUSB_ERROR_BUSY = 0x1016,                          //Resource busy
	STATUS_LIBUSB_ERROR_OTHER = 0x1000,							//Other error

	//STATUS_STREAM_ALREADY_EXISTS = 25,						//stream already exists
	//STATUS_UNSUPPORTED_STREAM = 26,							// unsupport stream

	STATUS_TIME_OUT = 102,
} Status;

/** The source of the stream */
typedef enum
{
	SENSOR_IR = 1,
	SENSOR_COLOR = 2,
	SENSOR_DEPTH = 3,

} SensorType;

/** All available formats of the output of a stream */
typedef enum
{
	// Depth
	PIXEL_FORMAT_DEPTH_1_MM = 100,
	PIXEL_FORMAT_DEPTH_100_UM = 101,
	PIXEL_FORMAT_SHIFT_9_2 = 102,
	PIXEL_FORMAT_SHIFT_9_3 = 103,

	// Color
	PIXEL_FORMAT_RGB888 = 200,
	PIXEL_FORMAT_YUV422 = 201,
	PIXEL_FORMAT_GRAY8 = 202,
	PIXEL_FORMAT_GRAY16 = 203,
	PIXEL_FORMAT_JPEG = 204,
	PIXEL_FORMAT_YUYV = 205,
	PIXEL_FORMAT_LOG = 207,
} PixelFormat;

typedef enum
{
	DEVICE_STATE_OK 	= 0,
	DEVICE_STATE_ERROR 	= 1,
	DEVICE_STATE_NOT_READY 	= 2,
	DEVICE_STATE_EOF 	= 3
} DeviceState;

typedef enum
{
	IMAGE_REGISTRATION_OFF				= 0,
	IMAGE_REGISTRATION_DEPTH_TO_COLOR	= 1,
} ImageRegistrationMode;

typedef enum
{
	PARAMS_REGISTRATION_OFF				= 0,
	PARAMS_REGISTRATION_DEPTH_TO_COLOR	= 1,
	PARAMS_REGISTRATION_USE_DISTORTION	= 2,
} ParamsRegistrationMode;

static const int TIMEOUT_NONE = 0;
static const int TIMEOUT_FOREVER = -1;

} // namespace openni

#endif // ONIENUMS_H
