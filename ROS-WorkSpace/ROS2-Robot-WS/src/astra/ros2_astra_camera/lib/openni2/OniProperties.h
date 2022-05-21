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
#ifndef ONIPROPERTIES_H
#define ONIPROPERTIES_H

namespace openni
{

// Device properties
enum
{
	DEVICE_PROPERTY_FIRMWARE_VERSION		= 0, // string
	DEVICE_PROPERTY_DRIVER_VERSION			= 1, // OniVersion
	DEVICE_PROPERTY_HARDWARE_VERSION		= 2, // int
	DEVICE_PROPERTY_SERIAL_NUMBER			= 3, // string
	DEVICE_PROPERTY_ERROR_STATE			= 4, // ??
	DEVICE_PROPERTY_IMAGE_REGISTRATION		= 5, // OniImageRegistrationMode

	 //orbbec
    OBEXTENSION_ID_IR_GAIN = 11,
    OBEXTENSION_ID_IR_EXP = 12,
    OBEXTENSION_ID_LDP_EN = 13,
    OBEXTENSION_ID_CAM_PARAMS = 14,
    OBEXTENSION_ID_LASER_EN = 15,
    OBEXTENSION_ID_SERIALNUMBER = 16,
    OBEXTENSION_ID_DEVICETYPE = 17,
    OBEXTENSION_ID_UPDATE_FIRMWARE = 18,
	// Files
	DEVICE_PROPERTY_PLAYBACK_SPEED			= 100, // float
	DEVICE_PROPERTY_PLAYBACK_REPEAT_ENABLED		= 101, // OniBool
};

// Stream properties
enum
{
	STREAM_PROPERTY_CROPPING			= 0, // OniCropping*
	STREAM_PROPERTY_HORIZONTAL_FOV			= 1, // float: radians
	STREAM_PROPERTY_VERTICAL_FOV			= 2, // float: radians
	STREAM_PROPERTY_VIDEO_MODE			= 3, // OniVideoMode*

	STREAM_PROPERTY_MAX_VALUE			= 4, // int
	STREAM_PROPERTY_MIN_VALUE			= 5, // int

	STREAM_PROPERTY_STRIDE				= 6, // int
	STREAM_PROPERTY_MIRRORING			= 7, // OniBool

	STREAM_PROPERTY_NUMBER_OF_FRAMES		= 8, // int

	// Camera
	STREAM_PROPERTY_AUTO_WHITE_BALANCE		= 100, // OniBool
	STREAM_PROPERTY_AUTO_EXPOSURE			= 101, // OniBool
	STREAM_PROPERTY_EXPOSURE				= 102, // int
	STREAM_PROPERTY_GAIN					= 103, // int

	STREAM_PROPERTY_SOFTWARE_REGISTRATION	= 0x2080FF42, // int
};

// Device commands (for Invoke)
enum
{
	DEVICE_COMMAND_SEEK				= 1, // OniSeek
};

} // namespace openni
#endif // ONIPROPERTIES_H
