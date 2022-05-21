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
#ifndef ONICENUMS_H
#define ONICENUMS_H

/** Possible failure values */
typedef enum
{
	ONI_STATUS_OK = 0,
	ONI_STATUS_ERROR = 1,
	ONI_STATUS_NOT_IMPLEMENTED = 2,
	ONI_STATUS_NOT_SUPPORTED = 3,
	ONI_STATUS_BAD_PARAMETER = 4,
	ONI_STATUS_OUT_OF_FLOW = 5,
	ONI_STATUS_NO_DEVICE = 6,
	ONI_STATUS_NOT_WRITE_PUBLIC_KEY = 7,
	ONI_STATUS_PUBLIC_KEY_MD5_VERIFY_FAILED = 8,
	ONI_STATUS_NOT_WRITE_MD5 =9,
	ONI_STATUS_RSKEY_VERIFY_FAILED = 10,

	//New add
	ONI_STATUS_DEVICE_IS_ALREADY_OPENED = 0x1001,                   //Device is already opened
	ONI_STATUS_USB_DRIVER_NOT_FOUND = 0x1002,                       //USB driver not found,for windows!
	ONI_STATUS_USB_DEVICE_NOT_FOUND = 0x1003,                       //USB device not found,for windows and linux
	ONI_STATUS_USB_GET_DRIVER_VERSION = 0x1004,                     //Failed to get the USB driver version,for windows!
	ONI_STATUS_USB_GET_SPEED_FAILED = 0x1005,                       //Failed to get the device speed,for windows!
	ONI_STATUS_USB_SET_INTERFACE_FAILED = 0x1006,                   //Failed to set USB interface!
	ONI_STATUS_USB_DEVICE_OPEN_FAILED = 0x1007,						//pid,vid,bus id is zero,for linux
	ONI_STATUS_USB_ENUMERATE_FAILED = 0x1008,						//USB enum fail,for linux
	ONI_STATUS_USB_SET_CONFIG_FAILED = 0x1009,						//usb set config fail ,for linux
	ONI_STATUS_USB_INIT_FAILED = 0x1010,							//libusb open fail
	ONI_STATUS_LIBUSB_ERROR_NO_MEM = 0x1011,						//Insufficient memory
	ONI_STATUS_LIBUSB_ERROR_ACCESS = 0x1012,						//Access denied (insufficient permissions)
	ONI_STATUS_LIBUSB_ERROR_NO_DEVICE = 0x1013,						//No such device (it may have been disconnected)
	ONI_STATUS_LIBUSB_ERROR_IO = 0x1014,							//Input/output error
	ONI_STATUS_LIBUSB_ERROR_NOT_FOUND = 0x1015,                     //Entity not found
	ONI_STATUS_LIBUSB_ERROR_BUSY = 0x1016,                          //Resource busy
	ONI_STATUS_LIBUSB_ERROR_OTHER = 0x1000,							//Other error

	//ONI_STATUS_STREAM_ALREADY_EXISTS = 25,						//stream already exists
	//ONI_STATUS_UNSUPPORTED_STREAM = 26,							// unsupport stream

	ONI_STATUS_TIME_OUT = 102,
} OniStatus;

/** The source of the stream */
typedef enum
{
	ONI_SENSOR_IR = 1,
	ONI_SENSOR_COLOR = 2,
	ONI_SENSOR_DEPTH = 3,

} OniSensorType;

/** All available formats of the output of a stream */
typedef enum
{
	// Depth
	ONI_PIXEL_FORMAT_DEPTH_1_MM = 100,
	ONI_PIXEL_FORMAT_DEPTH_100_UM = 101,
	ONI_PIXEL_FORMAT_SHIFT_9_2 = 102,
	ONI_PIXEL_FORMAT_SHIFT_9_3 = 103,

	// Color
	ONI_PIXEL_FORMAT_RGB888 = 200,
	ONI_PIXEL_FORMAT_YUV422 = 201,
	ONI_PIXEL_FORMAT_GRAY8 = 202,
	ONI_PIXEL_FORMAT_GRAY16 = 203,
	ONI_PIXEL_FORMAT_JPEG = 204,
	ONI_PIXEL_FORMAT_YUYV = 205,
	ONI_PIXEL_FORMAT_LOG = 207,
} OniPixelFormat;

typedef enum
{
	ONI_DEVICE_STATE_OK 		= 0,
	ONI_DEVICE_STATE_ERROR		= 1,
	ONI_DEVICE_STATE_NOT_READY 	= 2,
	ONI_DEVICE_STATE_EOF 		= 3
} OniDeviceState;

typedef enum
{
	ONI_IMAGE_REGISTRATION_OFF				= 0,
	ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR	= 1,
} OniImageRegistrationMode;

enum
{
	ONI_TIMEOUT_NONE = 0,
	ONI_TIMEOUT_FOREVER = -1,
};

#endif // ONICENUMS_H
