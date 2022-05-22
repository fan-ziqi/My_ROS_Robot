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
#ifndef PS1080_H
#define PS1080_H

#include <OniCTypes.h>

/** The maximum permitted Xiron device name string length. */ 
#define XN_DEVICE_MAX_STRING_LENGTH 200

/* 
 * private properties of PS1080 devices.
 *
 * @remarks 
 * properties structure is 0x1080XXYY where XX is range and YY is code.
 * range values:
 * F0 - device properties
 * E0 - device commands
 * 00 - common stream properties
 * 10 - depth stream properties
 * 20 - color stream properties
 */
enum
{
	/*******************************************************************/
	/* Device properties                                               */
	/*******************************************************************/

	/** unsigned long long (XnSensorUsbInterface) */
	XN_MODULE_PROPERTY_USB_INTERFACE = 0x1080F001, // "UsbInterface"
	/** Boolean */
	XN_MODULE_PROPERTY_MIRROR = 0x1080F002, // "Mirror"
	/** unsigned long long, get only */
	XN_MODULE_PROPERTY_RESET_SENSOR_ON_STARTUP = 0x1080F004, // "ResetSensorOnStartup"
	/** unsigned long long, get only */
	XN_MODULE_PROPERTY_LEAN_INIT = 0x1080F005, // "LeanInit"
	/** char[XN_DEVICE_MAX_STRING_LENGTH], get only */
	XN_MODULE_PROPERTY_SERIAL_NUMBER = 0x1080F006, // "ID"
	/** XnVersions, get only */
	XN_MODULE_PROPERTY_VERSION = 0x1080F007, // "Version"
	/** Boolean */
	XN_MODULE_PROPERTY_FIRMWARE_FRAME_SYNC = 0x1080F008,
	/** Boolean */
	XN_MODULE_PROPERTY_HOST_TIMESTAMPS = 0x1080FF77, // "HostTimestamps"
	/** Boolean */
	XN_MODULE_PROPERTY_CLOSE_STREAMS_ON_SHUTDOWN = 0x1080FF78, // "CloseStreamsOnShutdown"
	/** Integer */
	XN_MODULE_PROPERTY_FIRMWARE_LOG_INTERVAL = 0x1080FF7F, // "FirmwareLogInterval"
	/** Boolean */
	XN_MODULE_PROPERTY_PRINT_FIRMWARE_LOG = 0x1080FF80, // "FirmwareLogPrint"
	/** Integer */
	XN_MODULE_PROPERTY_FIRMWARE_LOG_FILTER = 0x1080FF81, // "FirmwareLogFilter"
	/** String, get only */
	XN_MODULE_PROPERTY_FIRMWARE_LOG = 0x1080FF82, // "FirmwareLog"
	/** Integer */
	XN_MODULE_PROPERTY_FIRMWARE_CPU_INTERVAL = 0x1080FF83, // "FirmwareCPUInterval"
	/** String, get only */
	XN_MODULE_PROPERTY_PHYSICAL_DEVICE_NAME = 0x1080FF7A, // "PhysicalDeviceName"
	/** String, get only */
	XN_MODULE_PROPERTY_VENDOR_SPECIFIC_DATA = 0x1080FF7B, // "VendorSpecificData"
	/** String, get only */
	XN_MODULE_PROPERTY_SENSOR_PLATFORM_STRING = 0x1080FF7C, // "SensorPlatformString"

	/*******************************************************************/
	/* Device commands (activated via SetProperty/GetProperty)         */
	/*******************************************************************/

	/** XnInnerParam */
	XN_MODULE_PROPERTY_FIRMWARE_PARAM = 0x1080E001, // "FirmwareParam"
	/** unsigned long long, set only */
	XN_MODULE_PROPERTY_RESET = 0x1080E002, // "Reset"
	/** XnControlProcessingData */
	XN_MODULE_PROPERTY_IMAGE_CONTROL = 0x1080E003, // "ImageControl"
	/** XnControlProcessingData */
	XN_MODULE_PROPERTY_DEPTH_CONTROL = 0x1080E004, // "DepthControl"
	/** XnAHBData */
	XN_MODULE_PROPERTY_AHB = 0x1080E005, // "AHB"
	/** XnLedState */
	XN_MODULE_PROPERTY_LED_STATE = 0x1080E006, // "LedState"
	/** Boolean */
	XN_MODULE_PROPERTY_EMITTER_STATE = 0x1080E007, // "EmitterState"

	/** XnCmosBlankingUnits */
	XN_MODULE_PROPERTY_CMOS_BLANKING_UNITS = 0x1080FF74, // "CmosBlankingUnits"
	/** XnCmosBlankingTime */
	XN_MODULE_PROPERTY_CMOS_BLANKING_TIME = 0x1080FF75, // "CmosBlankingTime"
	/** XnFlashFileList, get only */
	XN_MODULE_PROPERTY_FILE_LIST = 0x1080FF84, // "FileList"
	/** XnParamFlashData, get only */
	XN_MODULE_PROPERTY_FLASH_CHUNK = 0x1080FF85, // "FlashChunk"
	XN_MODULE_PROPERTY_FILE = 0x1080FF86, // "FlashFile"
	/** Integer */
	XN_MODULE_PROPERTY_DELETE_FILE = 0x1080FF87, // "DeleteFile"
	XN_MODULE_PROPERTY_FILE_ATTRIBUTES = 0x1080FF88, // "FileAttributes"
	XN_MODULE_PROPERTY_TEC_SET_POINT = 0x1080FF89, // "TecSetPoint"
	/** get only */
	XN_MODULE_PROPERTY_TEC_STATUS = 0x1080FF8A, // "TecStatus"
	/** get only */
	XN_MODULE_PROPERTY_TEC_FAST_CONVERGENCE_STATUS = 0x1080FF8B, // "TecFastConvergenceStatus"
	XN_MODULE_PROPERTY_EMITTER_SET_POINT = 0x1080FF8C, // "EmitterSetPoint"
	/** get only */
	XN_MODULE_PROPERTY_EMITTER_STATUS = 0x1080FF8D, // "EmitterStatus"
	XN_MODULE_PROPERTY_I2C = 0x1080FF8E, // "I2C"
	/** Integer, set only */
	XN_MODULE_PROPERTY_BIST = 0x1080FF8F, // "BIST"
	/** XnProjectorFaultData, set only */
	XN_MODULE_PROPERTY_PROJECTOR_FAULT = 0x1080FF90, // "ProjectorFault"
	/** Boolean, set only */
	XN_MODULE_PROPERTY_APC_ENABLED = 0x1080FF91, // "APCEnabled"
	/** Boolean */
	XN_MODULE_PROPERTY_FIRMWARE_TEC_DEBUG_PRINT = 0x1080FF92, // "TecDebugPrint"
	/** Boolean, set only */
	XN_MODULE_PROPERTY_READ_ALL_ENDPOINTS = 0x1080FF93,
	/*IRFLOOD*/
	XN_MODULE_PROPERTY_IRFLOOD_STATE = 0x1080FF94, // "Ir floodState"

	//irgain
	XN_MODULE_PROPERTY_IRGAIN = 0x1080FF95,  //Ir gain
	XN_MODULE_PROPERTY_IREXP = 0x1080FF96,  //Ir exp

	XN_MODULE_PROPERTY_SENSOR_CHANGE = 0x1080FF97,      //ado change sensor

	XN_MODULE_PROPERTY_PUBLIC_KEY = 0x1080FF98,		//public key and batch version
	XN_MODULE_PROPERTY_RANDOM_STRING = 0x1080FF99,		//get random init string

	XN_MODULE_PROPERTY_RS_KEY = 0x1080FF9A,            //rs key
	//laser secure
	XN_MODULE_PROPERTY_IS_SUPPORT_LASER_SECURE = 0x1080FF9B,  //is support laser secure
	XN_MODULE_PROPERTY_LASER_SECURE_STATUS = 0x1080FF9C,      //laser secure (open or close)
	XN_MODULE_PROPERTY_LASER_SECURE_KEEPALIVE = 0x1080FF9D,    //keepalive heart packet
	//laser current
	XN_MODULE_PROPERTY_LASER_CURRENT = 0x1080FF9E,

	//soft reset
	XN_MODULE_PROPERTY_SOFT_RESET = 0x1080FF9F,
	//dual camera switch left and right ir
	XN_MODULE_PROPERTY_SWITCH_IR = 0x1080FFB0,

	//rgb Ae mode
	XN_MODULE_PROPERTY_RGB_AE_MODE = 0x1080FFB1,

	//IR Temperature mode
	XN_MODULE_PROPERTY_CAL_IR_TEMP = 0x1080FFB2,
	//LDMP Temperature mode
	XN_MODULE_PROPERTY_CAL_LDMP_TEMP = 0x1080FFB3,
	//IR real time Temperature mode
	XN_MODULE_PROPERTY_RT_IR_TEMP = 0x1080FFB4,
	//LDMP real time Temperature mode
	XN_MODULE_PROPERTY_RT_LDMP_TEMP = 0x1080FFB5,
	//IR temperature compensation coefficient mode
	XN_MODULE_PROPERTY_IR_TEMP_COMP_CO = 0x1080FFB6,
	//Ldmp temperature compensation coefficient mode
	XN_MODULE_PROPERTY_LDMP_TEMP_COMP_CO = 0x1080FFB7,

	//Temperature comp
	XN_MODULE_PROPERTY_TEMP_COMP = 0x1080FFB8,

	//Depth optimization enabled state
	XN_MODULE_PROPERTY_DEPTH_OPTIM_STATE = 0x1080FFB9,

	//Depth optimization param
	XN_MODULE_PROPERTY_DEPTH_OPTIM_PARAM = 0x1080FFBA,

	//Distortion calibration param
	XN_MODULE_PROPERTY_DISTORTION_PARAM = 0x1080FFBB,

	//Distortion calibration enable
	XN_MODULE_PROPERTY_DISTORTION_STATE = 0x1080FFBC,

	XN_MODULE_PROPERTY_UPDATE_FIRMWARE_FLASH_CHUNK = 0x1080FFBD,

	XN_MODULE_PROPERTY_LDP_ENABLE = 0x1080FFBE,
	XN_MODULE_PROPERTY_EMITTER_STATE_V1 = 0x1080FFBF,   // "get Emitter enable"
	XN_MODULE_PROPERTY_LDP_SCALE = 0x1080FFC0,          //ldp scale
	XN_MODULE_PROPERTY_LDP_STATUS = 0x1080FFC1,         //ldp status
	XN_MODULE_PROPERTY_LDP_THRES_UP = 0x1080FFFB,       //ldp thres up
	XN_MODULE_PROPERTY_LDP_THRES_LOW = 0x1080FFFC,      //ldp thres low
	XN_MODULE_PROPERTY_LDP_NOIST_VALUE = 0x1080FFFD,    //ldp noise value

	//Device QN read/write 
	XN_MODULE_PROPERTY_QN_INFO = 0x1080FFC2,
	XN_MODULE_PROPERTY_QN_VERIFY = 0x1080FFC3,

	//public br version
	XN_MODULE_PROPERTY_PUBLIC_BOARD_VERSION = 0x1080FFC4,

	//mx6300 version
	XN_MODULE_PROPERTY_VERSION_MX6300 = 0x1080FFC5,
	//rgb resolution for d2c
	XN_MODULE_PROPERTY_D2C_RESOLUTION = 0x1080FFC6,
	XN_MODULE_PROPERTY_USB_SPEED = 0x1080FFC7,
	XN_MODULE_PROPERTY_PRODUCT_SERIAL_NUMBER = 0x1080FFC8,//product write sn(S1/T1)
	XN_MODULE_PROPERTY_CORE_BROAD_FLASH_ID = 0x1080FFC9,  //mipi project core broad status
	//ir flood level
	XN_MODULE_PROPERTY_IRFLOOD_LEVEL = 0x1080FFCA,
	XN_MODULE_PROPERTY_PRODUCT_NUMBER = 0x1080FFCB,

	//Auto AE
	XN_MODULE_PROPERTY_AE = 0x1080FFCC,
	//u1
	XN_MODULE_PROPERTY_MIPI_TEST = 0x1080FFCD,
	//get z0 and baseline
	XN_MODULE_PROPERTY_Z0_BASELINE = 0x1080FFCE,
	XN_MODULE_PROPERTY_I2C_READ_FLASH_MIPI = 0x1080FFCF,
	XN_MODULE_PROPERTY_CFG_SNPN = 0x1080FFD0,
	XN_MODULE_PROPERTY_CFG_PN = 0x1080FFD1,
	XN_MODULE_PROPERTY_WRITE_REF = 0x1080FFD2,
	XN_MODULE_PROPERTY_WRITE_SW_ALIGN_PARAM = 0x1080FFD3,
	XN_MODULE_PROPERTY_WRITE_HW_ALIGN_PARAM = 0x1080FFD4,
	XN_MODULE_PROPERTY_WRITE_HW_DISTORTION_PARAM = 0x1080FFD5,
	XN_MODULE_PROPERTY_IRS_MODEl = 0x1080FFD6,
	XN_MODULE_PROPERTY_RGBS_MODEl = 0x1080FFD7,

	XN_MODULE_PROPERTY_FLOOD_AE_OPTIONS = 0x1080FFD8,
	XN_MODULE_PROPERTY_EMITTER_AE_OPTIONS = 0x1080FFD9,

	//PD
	XN_MODULE_PROPERTY_PD_ENABLE_STATUS = 0x1080FFDA,
	XN_MODULE_PROPERTY_PD_ALERT_STATUS = 0x1080FFDB,
	XN_MODULE_PROPERTY_PD_UPPER_TLV = 0x1080FFDC,
	XN_MODULE_PROPERTY_PD_LOWER_TLV = 0x1080FFDD,
	XN_MODULE_PROPERTY_PD_CUR_TLV = 0x1080FFDE,
	//Bootloader protection status
	XN_MODULE_PROPERTY_BOOTLOADER_PTS =0x1080FFDF,

	// Depth config.
	XN_MODULE_PROPERTY_WRITE_DEPTH_CONFIG = 0x1080FFE0,

	//laser time cli tool function
	XN_MODULE_PROPERTY_LASER_TIME = 0x1080FFE1,
	//post filter threshold
	XN_MODULE_PROPERTY_POSTFILTER_THRESHOLD = 0x1080FFE2,
	//get zpps
	XN_MODULE_PROPERTY_ZPPS = 0x1080FFE3,

	XN_MODULE_PROPERTY_IRGAIN_FLASH = 0x1080FFE4,
	XN_MODULE_PROPERTY_IREXP_FLASH = 0x1080FFE5,
	XN_MODULE_PROPERTY_POSTFILTER_THRESHOLD_FLASH = 0x1080FFE6,
	XN_MODULE_PROPERTY_LASER_CURRENT_FLASH = 0x1080FFE7,
	XN_MODULE_PROPERTY_LASER_TIME_FLASH = 0x1080FFE8,
	XN_MODULE_PROPERTY_CUP_VERIFY_VERSION = 0x1080FFE9,


	XN_MODULE_PROPERTY_TOF_SENSOR_ENABLE = 0x1080FFEA,
	XN_MODULE_PROPERTY_TOF_SENSOR_MEA_RESULT = 0x1080FFEB,
	XN_MODULE_PROPERTY_TOF_SENSOR_APP_ID = 0x1080FFEC,
	XN_MODULE_PROPERTY_TOF_SENSOR_CAL = 0x1080FFED,
	XN_MODULE_PROPERTY_TOF_SENSOR_APP_START = 0x1080FFEE,
	XN_MODULE_PROPERTY_TOF_SENSOR_CAL_PARAMS = 0x1080FFEF,
	XN_MODULE_PROPERTY_TOF_SIMPLE_PERIOD = 0x1080FFF9,
	XN_MODULE_PROPERTY_DEPTH_IR_MODE = 0x1080FFF0,
	XN_MODULE_PROPERTY_TEMP_COMP_PARAMS = 0x1080FFF1,
	XN_MODULE_PROPERTY_MULTI_PARSE_REFERENCE = 0x1080FFF2,
	XN_MODULE_PROPERTY_THIRD_ROM_VERSION = 0x1080FFF3,
	XN_MODULE_PROPERTY_THIRD_CHIP_MODEL = 0x1080FFF4,
	XN_MODULE_PROPERTY_THIRD_SDK_VERSION = 0x1080FFF5,
	XN_MODULE_PROPERTY_HW_FRAME_SYC = 0x1080FFF6,
	XN_MODULE_PROPERTY_THIRD_LOGCAT = 0x1080FFF7,
	XN_MODULE_PROPERTY_THIRD_LOGCAT_LEVEL = 0x1080FFF8,
	XN_MODULE_PROPERTY_THIRD_ROM_DOWDLOAD = 0x1080FFFA,
	XN_MODULE_PROPERTY_THIRD_AGING_STATUS = 0x1080FFFE,
	XN_MODULE_PROPERTY_THIRD_AGING_TIME = 0x1080FFFF,
	XN_MODULE_PROPERTY_THIRD_AGING_RESULT = 0x10810000,
	XN_MODULE_PROPERTY_THIRD_ROM_DOWDLOAD_STATE = 0x10810001,
	XN_MODULE_PROPERTY_ANT_ALGORITHM_PARAMS = 0x10810002,
	XN_MODULE_PROPERTY_ANT_SECURITY_KEY = 0x10810003,
	XN_MODULE_PROPERTY_TEC_ENABLE = 0x10810004,
	XN_MODULE_PROPERTY_TOF_LONGOPEN_MODE = 0x10810005,
	XN_MODULE_PROPERTY_TOF_SINGLE_RANGING = 0x10810006,
	XN_MODULE_PROPERTY_SERIAL_PORT_ENABLE = 0x10810007,
	XN_MODULE_PROPERTY_HW_SYN_FPS = 0x10810008,
	XN_MODULE_PROPERTY_LASER_OVERCURRENT_PTS = 0x10810009,
	XN_MODULE_PROPERTY_THIRD_SAVE_LOG_ENABLE = 0x1081000A,
	XN_MODULE_PROPERTY_THIRD_SF_FRAME_SYNC_ENABLE = 0x1081000B,
	XN_MODULE_PROPERTY_DEVICE_PID = 0x1081000C,

	XN_MODULE_PROPERTY_DEVICE_SBG_ENABLE = 0x1081000D,
	XN_MODULE_PROPERTY_DEVICE_SBG_MODE = 0x1081000E,
	XN_MODULE_PROPERTY_DEVICE_IR_GAMMA_ENABLE = 0x1081000F,
	XN_MODULE_PROPERTY_DEVICE_IR_FACE_AE_ROI = 0x10810010,

	XN_MODULE_PROPERTY_ACTIVATION_CODE = 0x10810011,
	XN_MODULE_PROPERTY_ACTIVATION_STATUS = 0x10810012,
	XN_MODULE_PROPERTY_VERIFY_ACTIVATION_CODE = 0x10810013,
	XN_MODULE_PROPERTY_FAN_ENABLE = 0x10810014,
	XN_MODULE_PROPERTY_FAN_STATUS = 0x10810015,
	XN_MODULE_PROPERTY_TEC_SET_CURRENT = 0x10810016,
	XN_MODULE_PROPERTY_TEC_GET_CURRENT = 0x10810017,
	XN_MODULE_PROPERTY_FAN_F_ENABLE = 0x10810018,
	XN_MODULE_PROPERTY_FAN_F_STATUS = 0x10810019,
	XN_MODULE_PROPERTY_TEC_SET_CURRENT_F = 0x1081001A,
	XN_MODULE_PROPERTY_TEC_GET_CURRENT_F = 0x1081001B,
	XN_MODULE_PROPERTY_CFG_PN_11 = 0x1081001C,
	XN_MODULE_PROPERTY_CFG_SET_PN = 0x1081001D,
	XN_MODULE_PROPERTY_LASER_PWM = 0x1081001E,
	XN_MODULE_PROPERTY_GPM = 0x1081001F,



	/*******************************************************************/
	/* Common stream properties                                        */
	/*******************************************************************/

	/** unsigned long long */ 
	XN_STREAM_PROPERTY_INPUT_FORMAT = 0x10800001, // "InputFormat"
	/** unsigned long long (XnCroppingMode) */
	XN_STREAM_PROPERTY_CROPPING_MODE = 0x10800002, // "CroppingMode"

	/*******************************************************************/
	/* Depth stream properties                                         */
	/*******************************************************************/

	/** unsigned long long */
	XN_STREAM_PROPERTY_CLOSE_RANGE = 0x1080F003, // "CloseRange"
	/** unsigned long long */
	XN_STREAM_PROPERTY_FAST_ZOOM_CROP = 0x1080F009, // "FastZoomCrop"
	/** XnPixelRegistration - get only */
	XN_STREAM_PROPERTY_PIXEL_REGISTRATION = 0x10801001, // "PixelRegistration"
	/** XnPixelRegistration - get only */
	XN_STREAM_PROPERTY_PIXEL_C2D_REGISTRATION = 0x20801001, // "PixelRegistration"
	/** XnPixelRegistration - get only */
	XN_STREAM_PROPERTY_PIXEL_D2C_REGISTRATION = 0x20801002, // "PixelRegistration"
	/** unsigned long long */
	XN_STREAM_PROPERTY_WHITE_BALANCE_ENABLED = 0x10801002, // "WhiteBalancedEnabled"
	/** unsigned long long */ 
	XN_STREAM_PROPERTY_GAIN = 0x10801003, // "Gain"
	/** unsigned long long */ 
	XN_STREAM_PROPERTY_HOLE_FILTER = 0x10801004, // "HoleFilter"
	/** unsigned long long (XnProcessingType) */ 
	XN_STREAM_PROPERTY_REGISTRATION_TYPE = 0x10801005, // "RegistrationType"
	/** XnDepthAGCBin* */
	XN_STREAM_PROPERTY_AGC_BIN = 0x10801006, // "AGCBin"
	/** unsigned long long, get only */ 
	XN_STREAM_PROPERTY_CONST_SHIFT = 0x10801007, // "ConstShift"
	/** unsigned long long, get only */ 
	XN_STREAM_PROPERTY_PIXEL_SIZE_FACTOR = 0x10801008, // "PixelSizeFactor"
	/** unsigned long long, get only */ 
	XN_STREAM_PROPERTY_MAX_SHIFT = 0x10801009, // "MaxShift"
	/** unsigned long long, get only */ 
	XN_STREAM_PROPERTY_PARAM_COEFF = 0x1080100A, // "ParamCoeff"
	/** unsigned long long, get only */ 
	XN_STREAM_PROPERTY_SHIFT_SCALE = 0x1080100B, // "ShiftScale"
	/** unsigned long long, get only */ 
	XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE = 0x1080100C, // "ZPD"
	/** double, get only */ 
	XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE = 0x1080100D, // "ZPPS"
	/** double, get only */ 
	XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE = 0x1080100E, // "LDDIS"
	/** double, get only */ 
	XN_STREAM_PROPERTY_DCMOS_RCMOS_DISTANCE = 0x1080100F, // "DCRCDIS"
	/*dual sensor */
	XN_STREAM_PROPERTY_DUAL_FOCALL_LENGTH = 0x06010011, // "DCRCDIS"
	/*dual sensor */
	XN_STREAM_PROPERTY_DUAL_COEFF_DISPARITY = 0x06010012, // "DCRCDIS"
	/** OniDepthPixel[], get only */ 
	XN_STREAM_PROPERTY_S2D_TABLE = 0x10801112, // "S2D"
	/** unsigned short[], get only */ 
	XN_STREAM_PROPERTY_D2S_TABLE = 0x10801113, // "D2S"
	/** get only */
	XN_STREAM_PROPERTY_DEPTH_SENSOR_CALIBRATION_INFO = 0x10801012,
	/** Boolean */
	XN_STREAM_PROPERTY_GMC_MODE	= 0x1080FF44, // "GmcMode"

    XN_STREAM_PROPERTY_MIN_DEPTH  = 0x1080FF40, // "MinDepthValue"
    XN_STREAM_PROPERTY_MAX_DEPTH  = 0x1080FF41, // "MaxDepthValue"
	/** Boolean */
	XN_STREAM_PROPERTY_GMC_DEBUG = 0x1080FF45, // "GmcDebug"
	/** Boolean */
	XN_STREAM_PROPERTY_WAVELENGTH_CORRECTION = 0x1080FF46, // "WavelengthCorrection"
	/** Boolean */
	XN_STREAM_PROPERTY_WAVELENGTH_CORRECTION_DEBUG = 0x1080FF47, // "WavelengthCorrectionDebug"

    /** Boolean */
    XN_STREAM_PROPERTY_SOFTWARE_REGISTRATION = 0x2080FF42, // "Software Registration"
	XN_STREAM_PROPERTY_SOFTWARE_FILTER = 0x2080FF43,		//soft filter enable
	XN_STREAM_PROPERTY_DEPTH_ROTATE = 0x2080FF44, 			//atlas depth rotate
	XN_STREAM_PROPERTY_DEPTH_MAX_DIFF = 0x2080FF45, 		//soft filter maxdiff param
	XN_STREAM_PROPERTY_DEPTH_MAX_SPECKLE_SIZE = 0x2080FF46, 	//soft filter maxSpeckleSize
	XN_STREAM_PROPERTY_DEPTH_UNIT_COEFFICIENT = 0x2080FF47, 	//Calibration unit accuracy coefficient
	XN_STREAM_PROPERTY_DEPTH_LEFT_EXTEND = 0x2080FF48, 	//Calibration unit accuracy coefficient
	XN_STREAM_PROPERTY_DEPTH_ORIGINAL_SHIFT = 0x2080FF49, 	//original shift or openni shift
	XN_STREAM_PROPERTY_DEPTH_SHIFT_COMPENSATION = 0x2080FF4A, //shift compensation
	/*******************************************************************/
	/* Color stream properties                                         */
	/*******************************************************************/
	/** Integer */ 
	XN_STREAM_PROPERTY_FLICKER = 0x10802001, // "Flicker"
};

typedef enum 
{
	XN_SENSOR_FW_VER_UNKNOWN = 0,
	XN_SENSOR_FW_VER_0_17 = 1,
	XN_SENSOR_FW_VER_1_1 = 2,
	XN_SENSOR_FW_VER_1_2 = 3,
	XN_SENSOR_FW_VER_3_0 = 4,
	XN_SENSOR_FW_VER_4_0 = 5,
	XN_SENSOR_FW_VER_5_0 = 6,
	XN_SENSOR_FW_VER_5_1 = 7,
	XN_SENSOR_FW_VER_5_2 = 8,
	XN_SENSOR_FW_VER_5_3 = 9,
	XN_SENSOR_FW_VER_5_4 = 10,
	XN_SENSOR_FW_VER_5_5 = 11,
	XN_SENSOR_FW_VER_5_6 = 12,
	XN_SENSOR_FW_VER_5_7 = 13,
	XN_SENSOR_FW_VER_5_8 = 14,
	XN_SENSOR_FW_VER_5_9 = 15,
} XnFWVer;

typedef enum {
	XN_SENSOR_VER_UNKNOWN = 0,
	XN_SENSOR_VER_2_0 = 1,
	XN_SENSOR_VER_3_0 = 2,
	XN_SENSOR_VER_4_0 = 3,
	XN_SENSOR_VER_5_0 = 4
} XnSensorVer;

typedef enum {
	XN_SENSOR_HW_VER_UNKNOWN = 0,
	XN_SENSOR_HW_VER_FPDB_10 = 1,
	XN_SENSOR_HW_VER_CDB_10  = 2,
	XN_SENSOR_HW_VER_RD_3  = 3,
	XN_SENSOR_HW_VER_RD_5  = 4,
	XN_SENSOR_HW_VER_RD1081  = 5,
	XN_SENSOR_HW_VER_RD1082  = 6,
	XN_SENSOR_HW_VER_RD109  = 7	
} XnHWVer;

typedef enum {
	XN_SENSOR_CHIP_VER_UNKNOWN = 0,
	XN_SENSOR_CHIP_VER_PS1000 = 1,
	XN_SENSOR_CHIP_VER_PS1080 = 2,
	XN_SENSOR_CHIP_VER_PS1080A6 = 3,
	XN_SENSOR_CHIP_VER_MX6000 = 6,
	XN_SENSOR_CHIP_VER_DUAL_MX6000 = 7

} XnChipVer;

typedef enum
{
	XN_CMOS_TYPE_IMAGE = 0,
	XN_CMOS_TYPE_DEPTH = 1,
	XN_CMOS_TYPE_IR = 2,
	XN_CMOS_COUNT
} XnCMOSType;

typedef enum
{
	XN_IO_IMAGE_FORMAT_BAYER = 0,
	XN_IO_IMAGE_FORMAT_YUV422 = 1,
	XN_IO_IMAGE_FORMAT_JPEG = 2,
	XN_IO_IMAGE_FORMAT_JPEG_420 = 3,
	XN_IO_IMAGE_FORMAT_JPEG_MONO = 4,
	XN_IO_IMAGE_FORMAT_UNCOMPRESSED_YUV422 = 5,
	XN_IO_IMAGE_FORMAT_UNCOMPRESSED_BAYER = 6,
	XN_IO_IMAGE_FORMAT_UNCOMPRESSED_YUYV = 7,
	XN_IO_IMAGE_FORMAT_MJPEG = 8,
	XN_IO_IMAGE_FORMAT_PLATFORM_LOG = 10,
} XnIOImageFormats;

typedef enum
{
	XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT = 0,
	XN_IO_DEPTH_FORMAT_COMPRESSED_PS = 1,
	XN_IO_DEPTH_FORMAT_UNCOMPRESSED_10_BIT = 2,
	XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT = 3,
	XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT = 4,

	//New addition
	XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT_DEPTH = 5,      //No compression 16 bit depth, no parallax depth
	XN_IO_DEPTH_FORMAT_COMPRESSED_10_BIT = 6,              //Compress 10bit parallax
	XN_IO_DEPTH_FORMAT_COMPRESSED_11_BIT = 7,              //Compress 11bit parallax
	XN_IO_DEPTH_FORMAT_COMPRESSED_12_BIT = 8,              //Compress 12bit parallax
	XN_IO_DEPTH_FORMAT_COMPRESSED_16_BIT = 9,              //Compress 16bit parallax
	XN_IO_DEPTH_FORMAT_COMPRESSED_16_BIT_DEPTH = 10,       //Compress 16bit depth without parallax depth

	//Standard Mipi packing raw data unpacking type is supported,
	//non-standard Mipi packing data unpacking is not supported
	XN_IO_DEPTH_FORMAT_UNCOMPRESSED_10_BIT_MIPI = 11,      //uncompressed 10bit Mipi package parallax
	XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT_MIPI = 12,      //uncompressed 11bit Mipi package parallax
	XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT_MIPI = 13,      //uncompressed 12bit Mipi package parallax

} XnIODepthFormats;

typedef enum
{
	XN_IO_IR_FORMAT_UNCOMPRESSED_16_BIT = 0,
	XN_IO_IR_FORMAT_COMPRESSED_PS = 1,
	XN_IO_IR_FORMAT_UNCOMPRESSED_10_BIT = 2,
	XN_IO_IR_FORMAT_UNCOMPRESSED_11_BIT = 3,
	XN_IO_IR_FORMAT_UNCOMPRESSED_12_BIT = 4,

	//New addition
	XN_IO_IR_FORMAT_COMPRESSED_16_BIT = 5,             //Compress 16bit
	XN_IO_IR_FORMAT_COMPRESSED_10_BIT = 6,             //Compress 10bit
	XN_IO_IR_FORMAT_COMPRESSED_11_BIT = 7,             //Compress 11bit
	XN_IO_IR_FORMAT_COMPRESSED_12_BIT = 8,             //Compress 18bit
} XnIOIRFormats;

typedef enum
{
	XN_RESET_TYPE_POWER = 0,
	XN_RESET_TYPE_SOFT = 1,
	XN_RESET_TYPE_SOFT_FIRST = 2,
} XnParamResetType;

typedef enum XnSensorUsbInterface
{
	XN_SENSOR_USB_INTERFACE_DEFAULT = 0,
	XN_SENSOR_USB_INTERFACE_ISO_ENDPOINTS = 1,
	XN_SENSOR_USB_INTERFACE_BULK_ENDPOINTS = 2,
	XN_SENSOR_USB_INTERFACE_ISO_ENDPOINTS_LOW_DEPTH = 3,
} XnSensorUsbInterface;

typedef enum XnProcessingType
{
	XN_PROCESSING_DONT_CARE = 0,
	XN_PROCESSING_HARDWARE = 1,
	XN_PROCESSING_SOFTWARE = 2,
} XnProcessingType;

typedef enum XnCroppingMode
{
	XN_CROPPING_MODE_NORMAL = 1,
	XN_CROPPING_MODE_INCREASED_FPS = 2,
	XN_CROPPING_MODE_SOFTWARE_ONLY = 3,
} XnCroppingMode;

enum
{
	XN_ERROR_STATE_OK = 0,
	XN_ERROR_STATE_DEVICE_PROJECTOR_FAULT = 1,
	XN_ERROR_STATE_DEVICE_OVERHEAT = 2,
};

typedef enum XnFirmwareCroppingMode
{
	XN_FIRMWARE_CROPPING_MODE_DISABLED = 0,
	XN_FIRMWARE_CROPPING_MODE_NORMAL = 1,
	XN_FIRMWARE_CROPPING_MODE_INCREASED_FPS = 2,
} XnFirmwareCroppingMode;

typedef enum
{
	XnLogFilterDebug		= 0x0001,
	XnLogFilterInfo			= 0x0002,
	XnLogFilterError		= 0x0004,
	XnLogFilterProtocol		= 0x0008,
	XnLogFilterAssert		= 0x0010,
	XnLogFilterConfig		= 0x0020,
	XnLogFilterFrameSync	= 0x0040,
	XnLogFilterAGC			= 0x0080,
	XnLogFilterTelems		= 0x0100,

	XnLogFilterAll			= 0xFFFF
} XnLogFilter;

typedef enum
{
	XnFileAttributeReadOnly	= 0x8000
} XnFilePossibleAttributes;

typedef enum
{
	XnFlashFileTypeFileTable					= 0x00,
	XnFlashFileTypeScratchFile					= 0x01,
	XnFlashFileTypeBootSector					= 0x02,
	XnFlashFileTypeBootManager					= 0x03,
	XnFlashFileTypeCodeDownloader				= 0x04,
	XnFlashFileTypeMonitor						= 0x05,
	XnFlashFileTypeApplication					= 0x06,
	XnFlashFileTypeFixedParams					= 0x07,
	XnFlashFileTypeDescriptors					= 0x08,
	XnFlashFileTypeDefaultParams				= 0x09,
	XnFlashFileTypeImageCmos					= 0x0A,
	XnFlashFileTypeDepthCmos					= 0x0B,
	XnFlashFileTypeAlgorithmParams				= 0x0C,
	XnFlashFileTypeReferenceQVGA				= 0x0D,
	XnFlashFileTypeReferenceVGA					= 0x0E,
	XnFlashFileTypeMaintenance					= 0x0F,
	XnFlashFileTypeDebugParams					= 0x10,
	XnFlashFileTypePrimeProcessor				= 0x11,
	XnFlashFileTypeGainControl					= 0x12,
	XnFlashFileTypeRegistartionParams			= 0x13,
	XnFlashFileTypeIDParams						= 0x14,
	XnFlashFileTypeSensorTECParams				= 0x15,
	XnFlashFileTypeSensorAPCParams				= 0x16,
	XnFlashFileTypeSensorProjectorFaultParams	= 0x17,
	XnFlashFileTypeProductionFile				= 0x18,
	XnFlashFileTypeUpgradeInProgress			= 0x19,
	XnFlashFileTypeWavelengthCorrection			= 0x1A,
	XnFlashFileTypeGMCReferenceOffset			= 0x1B,
	XnFlashFileTypeSensorNESAParams				= 0x1C,
	XnFlashFileTypeSensorFault					= 0x1D,
	XnFlashFileTypeVendorData					= 0x1E,
} XnFlashFileType;

typedef enum XnBistType
{
	//Auto tests
	XN_BIST_IMAGE_CMOS = 1 << 0,
	XN_BIST_IR_CMOS = 1 << 1,
	XN_BIST_POTENTIOMETER = 1 << 2,
	XN_BIST_FLASH = 1 << 3,
	XN_BIST_FULL_FLASH = 1 << 4,
	XN_BIST_PROJECTOR_TEST_MASK = 1 << 5,
	XN_BIST_TEC_TEST_MASK = 1 << 6,

	// Manual tests
	XN_BIST_NESA_TEST_MASK = 1 << 7,
	XN_BIST_NESA_UNLIMITED_TEST_MASK = 1 << 8,

	// Mask of all the auto tests
	XN_BIST_ALL = (0xFFFFFFFF & ~XN_BIST_NESA_TEST_MASK & ~XN_BIST_NESA_UNLIMITED_TEST_MASK),

} XnBistType;

typedef enum XnBistError
{
	XN_BIST_RAM_TEST_FAILURE = 1 << 0,
	XN_BIST_IR_CMOS_CONTROL_BUS_FAILURE = 1 << 1,
	XN_BIST_IR_CMOS_DATA_BUS_FAILURE = 1 << 2,
	XN_BIST_IR_CMOS_BAD_VERSION = 1 << 3,
	XN_BIST_IR_CMOS_RESET_FAILUE = 1 << 4,
	XN_BIST_IR_CMOS_TRIGGER_FAILURE = 1 << 5,
	XN_BIST_IR_CMOS_STROBE_FAILURE = 1 << 6,
	XN_BIST_COLOR_CMOS_CONTROL_BUS_FAILURE = 1 << 7,
	XN_BIST_COLOR_CMOS_DATA_BUS_FAILURE = 1 << 8,
	XN_BIST_COLOR_CMOS_BAD_VERSION = 1 << 9,
	XN_BIST_COLOR_CMOS_RESET_FAILUE = 1 << 10,
	XN_BIST_FLASH_WRITE_LINE_FAILURE = 1 << 11,
	XN_BIST_FLASH_TEST_FAILURE = 1 << 12,
	XN_BIST_POTENTIOMETER_CONTROL_BUS_FAILURE = 1 << 13,
	XN_BIST_POTENTIOMETER_FAILURE = 1 << 14,
	XN_BIST_AUDIO_TEST_FAILURE = 1 << 15,
	XN_BIST_PROJECTOR_TEST_LD_FAIL = 1 << 16,
	XN_BIST_PROJECTOR_TEST_LD_FAILSAFE_TRIG_FAIL = 1 << 17,
	XN_BIST_PROJECTOR_TEST_FAILSAFE_HIGH_FAIL = 1 << 18,
	XN_BIST_PROJECTOR_TEST_FAILSAFE_LOW_FAIL = 1 << 19,
	XN_TEC_TEST_HEATER_CROSSED = 1 << 20,
	XN_TEC_TEST_HEATER_DISCONNETED = 1 << 21,
	XN_TEC_TEST_TEC_CROSSED = 1 << 22,
	XN_TEC_TEST_TEC_FAULT = 1 << 23,
} XnBistError;

typedef enum XnDepthCMOSType
{
	XN_DEPTH_CMOS_NONE = 0,
	XN_DEPTH_CMOS_MT9M001 = 1,
	XN_DEPTH_CMOS_AR130 = 2,
	XN_DEPTH_CMOS_OV9151 = 3,

	XN_DEPTH_CMOS_OV9282 = 7,
	XN_DEPTH_CMOS_AR0144 = 8,
} XnDepthCMOSType;

typedef enum XnImageCMOSType
{
	XN_IMAGE_CMOS_NONE = 0,
	XN_IMAGE_CMOS_MT9M112 = 1,
	XN_IMAGE_CMOS_MT9D131 = 2,
	XN_IMAGE_CMOS_MT9M114 = 3,
} XnImageCMOSType;

typedef enum {
	XN_USB_LOW_SPEED = 0,
	XN_USB_FULL_SPEED,
	XN_USB_HIGH_SPEED,
	XN_USB_SUPER_SPEED,
} XnUSBSpeed;

#define XN_IO_MAX_I2C_BUFFER_SIZE 10
#define XN_MAX_LOG_SIZE	(6*1024)

#pragma pack (push, 1)

typedef struct XnSDKVersion
{
	unsigned char nMajor;
	unsigned char nMinor;
	unsigned char nMaintenance;
	unsigned short nBuild;
} XnSDKVersion;

typedef struct {
	unsigned char nMajor;
	unsigned char nMinor;
	unsigned short nBuild;
	unsigned int nChip;
	unsigned short nFPGA;
	unsigned short nSystemVersion;

	XnSDKVersion SDK;

	XnHWVer		HWVer;
	XnFWVer		FWVer;
	XnSensorVer SensorVer;
	XnChipVer	ChipVer;
} XnVersions;

typedef struct
{
	unsigned short nParam;
	unsigned short nValue;
} XnInnerParamData;

typedef struct XnDepthAGCBin 
{
	unsigned short nBin;
	unsigned short nMin;
	unsigned short nMax;
} XnDepthAGCBin;

typedef struct XnControlProcessingData
{
	unsigned short nRegister;
	unsigned short nValue;
} XnControlProcessingData;

typedef struct XnAHBData
{
	unsigned int nRegister;
	unsigned int nValue;
	unsigned int nMask;
} XnAHBData;

typedef struct XnPixelRegistration
{
	unsigned int nDepthX;
	unsigned int nDepthY;
	uint16_t nDepthValue;
	unsigned int nImageXRes;
	unsigned int nImageYRes;
	unsigned int nImageX; // out
	unsigned int nImageY; // out
} XnPixelRegistration;

typedef struct XnLedState
{
	uint16_t nLedID;
	uint16_t nState;
} XnLedState;


typedef struct XnCmosBlankingTime
{
	XnCMOSType nCmosID;
	float nTimeInMilliseconds;
	uint16_t nNumberOfFrames;
} XnCmosBlankingTime;

typedef struct XnCmosBlankingUnits
{
	XnCMOSType nCmosID;
	uint16_t nUnits;
	uint16_t nNumberOfFrames;
} XnCmosBlankingUnits;

typedef struct XnI2CWriteData
{
	uint16_t nBus;
	uint16_t nSlaveAddress;
	uint16_t cpWriteBuffer[XN_IO_MAX_I2C_BUFFER_SIZE];
	uint16_t nWriteSize;
} XnI2CWriteData;

typedef struct XnI2CReadData
{
	uint16_t nBus;
	uint16_t nSlaveAddress;
	uint16_t cpReadBuffer[XN_IO_MAX_I2C_BUFFER_SIZE];
	uint16_t cpWriteBuffer[XN_IO_MAX_I2C_BUFFER_SIZE];
	uint16_t nReadSize;
	uint16_t nWriteSize;
} XnI2CReadData;

typedef struct XnTecData
{
	uint16_t m_SetPointVoltage;
	uint16_t m_CompensationVoltage;
	uint16_t m_TecDutyCycle; //duty cycle on heater/cooler
	uint16_t m_HeatMode; //TRUE - heat, FALSE - cool
	int32_t m_ProportionalError;
	int32_t m_IntegralError;
	int32_t m_DerivativeError;
	uint16_t m_ScanMode; //0 - crude, 1 - precise
} XnTecData;

typedef struct XnTecFastConvergenceData
{
	int16_t     m_SetPointTemperature;  // set point temperature in celsius,
	// scaled by factor of 100 (extra precision)
	int16_t     m_MeasuredTemperature;  // measured temperature in celsius,
	// scaled by factor of 100 (extra precision)
	int32_t 	m_ProportionalError;    // proportional error in system clocks
	int32_t 	m_IntegralError;        // integral error in system clocks
	int32_t 	m_DerivativeError;      // derivative error in system clocks
	uint16_t 	m_ScanMode; // 0 - initial, 1 - crude, 2 - precise
	uint16_t    m_HeatMode; // 0 - idle, 1 - heat, 2 - cool
	uint16_t    m_TecDutyCycle; // duty cycle on heater/cooler in percents
	uint16_t	m_TemperatureRange;	// 0 - cool, 1 - room, 2 - warm
} XnTecFastConvergenceData;

typedef struct XnEmitterData
{
	uint16_t m_State; //idle, calibrating
	uint16_t m_SetPointVoltage; //this is what should be written to the XML
	uint16_t m_SetPointClocks; //target cross duty cycle
	uint16_t m_PD_Reading; //current cross duty cycle in system clocks(high time)
	uint16_t m_EmitterSet; //duty cycle on emitter set in system clocks (high time).
	uint16_t m_EmitterSettingLogic; //TRUE = positive logic, FALSE = negative logic
	uint16_t m_LightMeasureLogic; //TRUE - positive logic, FALSE - negative logic
	uint16_t m_IsAPCEnabled;
	uint16_t m_EmitterSetStepSize; // in MilliVolts
	uint16_t m_ApcTolerance; // in system clocks (only valid up till v5.2)
	uint16_t m_SubClocking; //in system clocks (only valid from v5.3)
	uint16_t m_Precision; // (only valid from v5.3)
} XnEmitterData;

typedef struct
{
	uint16_t nId;
	uint16_t nAttribs;
} XnFileAttributes;

typedef struct
{
	uint32_t nOffset;
	const char* strFileName;
	uint16_t nAttributes;
} XnParamFileData;

typedef struct
{
	uint32_t nOffset;
	uint32_t nSize;
	unsigned char* pData;
} XnParamFlashData;

typedef struct  {
	uint16_t nId;
	uint16_t nType;
	uint32_t nVersion;
	uint32_t nOffset;
	uint32_t nSize;
	uint16_t nCrc;
	uint16_t nAttributes;
	uint16_t nReserve;
} XnFlashFile;

typedef struct  
{
	XnFlashFile* pFiles;
	uint16_t nFiles;
} XnFlashFileList;

typedef struct XnProjectorFaultData
{
	uint16_t nMinThreshold;
	uint16_t nMaxThreshold;
	int32_t bProjectorFaultEvent;
} XnProjectorFaultData;

typedef struct XnBist
{
	uint32_t nTestsMask;
	uint32_t nFailures;
} XnBist;

//public key and batch version
typedef struct OBEccVerify
{
	char Pub_x[48];			//public key x 48 bytes
	char Pub_y[48];         //public key y 48 bytes
	char Batch_Ver[12];		//batch version 12 bytes
}OBEccVerify;

//get init random string
typedef struct OBEccInit
{
	char RandomStr[48];
	char Batch_Ver[12];
}OBEccInit;

//r s key
typedef struct OBEccRSKey
{
	char R_key[48];
	char S_key[48];
}OBEccRSKey;

//rgb ae mode
typedef struct XnRgbAeMode
{
	uint16_t nAeMode;
	uint16_t nAeTarget;
} XnRgbAeMode;

typedef struct XnDepthOptimizationParam
{
	double nParam1;
	double nParam2;
	double nParam3;
}XnDepthOptimizationParam;

typedef struct {
	uint32_t nSize;
	unsigned char* data;
}XnDistortionParam;


typedef struct OBFirmwareQN
{
	uint8_t QN[8];
}OBFirmwareQN;

typedef struct OBPublicBoardVersion
{
	uint8_t cVersion[4];
}OBPublicBoardVersion;

typedef struct ObMX6300Version
{
	uint8_t cVersion[4];
} ObMX6300Version;

typedef struct OBSerialNumber
{
	uint32_t size;
	uint8_t SN[32];
}OBSerialNumber;

typedef struct OBKTProductNumber
{
	uint8_t PN[32];
}OBKTProductNumber;

typedef struct OBZ0Baseline
{
	float fZ0;
	float fBaseline;
}OBZ0Baseline;
typedef struct OBCfgSerialProductNumber
{
	uint8_t SerialNumber[12];
	uint8_t ProductNumber[12];
}OBCfgSerialProductNumber;

typedef struct {
	uint32_t nRegNum;
	uint32_t nSize;
	unsigned char data[64*1024];
}OBCfgHwD2cDistortion;

#define OB_AUTHORIZATION_CODE_SIZE 16
#define OB_ACTIVATION_CODE_SIZE 49

typedef struct OBAuthorizationCode{
	uint8_t AuthCode[OB_AUTHORIZATION_CODE_SIZE];
}OBAuthorizationCode;

typedef struct OBActivationCode{
	uint8_t ActiveCode[OB_ACTIVATION_CODE_SIZE];
}OBActivationCode;

enum ObDeviceActivationStatus
{
	DEVICE_ACTIVATION_STATUS_DEACTIVATED = 0,
	DEVICE_ACTIVATION_STATUS_ACTIVATED = 1,
};

typedef struct OBPdThreshold
{
	uint32_t PdThreshold;
}OBPdThreshold;

enum AEOPTIONPARAMES
{
	EXPTIME = 0,
	AGAIN, 
	LASERCURRENT, 
	TARGETBRIGHTNESS,
	TARGETTHD,
	CENTERWEIGHT,
	SKIPFRAME, 
	SMOOTHSTEPS,
	DELAYMS,
	METERMETHOD,
	EXPTIMEADJ, 
	AGAINADJ, 
	LASERCURRENTADJ,
};

typedef struct _Range
{
	uint32_t min;
	uint32_t max;
}RANGE;

enum METER_METHOD
{
	CENTER_METERING = 0,
	AVERRAGE_METERING,
	SPOT_METERING,
	ROI_METERING,
	AUTO_METERING,
	WEIGHT_METERING,
	INV_TRIANGLE,
};
typedef struct
{
	RANGE expTime;
	RANGE AGain;
	RANGE laserCurrent;
	uint32_t targetBrightness;
	uint32_t targetThd;
	uint32_t centerWeight;
	uint32_t skipFrame;
	uint32_t smoothSteps;
	uint32_t dealy_ms;
	uint32_t meterMethod;
	uint8_t expTimeAdj;
	uint8_t AGainAdj;
	uint8_t laserCurrentAdj;
	uint8_t reserve;
} AeParamsStruct;

//china union pay certification
typedef struct
{
	uint8_t ProductType[32];
	uint8_t FirewareVersion[4];
}CupCertify;


typedef struct OBTofSensorCalParams
{
	uint8_t calParams[14];
}OBTofSensorCalParams;

typedef struct OBTempParams
{
	uint32_t temp_Enable;       //Enable or disable temperature compensation.
	float temp_IR;         //Real-time temperature of IR module.
	float temp_Laser;      //Real-time temperature of Laser module.
	float temp_Cal_IR;     //Calibration temperature of IR module.
	float temp_Cal_Laser;  //Calibration temperature of Laser module.
	float ncost_IR;        //Temperature coefficient of IR module.
	float ncost_Laser;     //Temperature coefficient of Laser module.
} OBTempParams;


typedef struct {
	uint32_t nSize;
	unsigned char* data;
}OBMultiParseRef;

typedef struct {
	uint32_t nSize;
	unsigned char* data;
}OBThirdRomBuffer;

enum THIRD_LOG_LEVEL
{
	THIRD_LOG_LEVEL_NO = 0,
	THIRD_LOG_LEVEL_ERROR,
	THIED_LOG_LEVEL_WARNING,
	THIRD_LOG_LEVEL_INFO,
};

enum THIRD_AGING_STATE
{
	THIRD_DEVICE_AGING_STOP = 0,  //Stop aging
	THIRD_DEVICE_AGING_START = 1, //Start aging
};

typedef enum
{
	PROTOCOL_DEPTH_NIR = 0,     //Depth + pure IR interleaving (USB device default mode)
	PROTOCOL_SPECKLE_IR = 1,    //Speckle IR
	PROTOCOL_NIR = 2,           //Floodlight (pure) IR
	PROTOCOL_IR_NIR = 3,        //nterleaved output of speckle IR and pure IR
} OBDepthIrMode;

enum THREAD_UPDATE_STATE
{
	THIRD_UPDATE_STATE_COMPLETE = 0,                                      //upgrade completed and succeeded
	THREAD_UPDATE_STATE_IN_PROCESSING = 1,                                //Upgrade in progress
	THREAD_UPDATE_STATE_NO_START = -1,                                    //the upgrade has not started yet
	THREAD_UPDATE_STATE_SAVE_ROM_FILE_FAILED = -2,                        //upgrade write save ROM file failed
	THREAD_UPDATE_STATE_SAVE_WRITE_END_MARK_FILE_FAILED = -3,             //upgrade write save end flag file failed
};

typedef struct
{
	uint32_t nSize;
	unsigned char* pData;
} AntAlgorithmParams;

typedef struct
{
	uint32_t keyPos;
	uint32_t keyValues[8];
} AntSecurityKey;

typedef struct
{
	uint32_t keyPos;
	int32_t  keyStatus;
} AntSecurityKeyStatus;

typedef enum
{
	DEPTH_IR_FPS_15 = 0,    //Firmware depth, IR 15 frame output
	DEPTH_IR_FPS_30 = 1,    //Firmware depth, IR 30 frame output
} HardwareFrameSynFPS;

typedef enum
{
	SBG_MODE_0 = 0,
	SBG_MODE_1 = 1,
	SBG_MODE_2 = 2,
} SBG_MODE;

typedef struct
{
	uint32_t nEnable;  //1:enable ir faceAE;0: disable ir faceAE
	uint16_t nLeft;	   
	uint16_t nTop;
	uint16_t nWidth;
	uint16_t nHeight;
}IrFaceAERoi;

typedef struct
{
	uint32_t mode;        //mode:0~3
	uint32_t dutyRatio;   //Duty ratio 0 ~ 99
	uint32_t frequency;   //Frequency: 10Hz ~ 15000hz
} LaserPWM;


typedef struct
{
	LaserPWM pwm0;
	LaserPWM pwm2;
	LaserPWM pwm3;
} LaserPWMS;


#pragma pack (pop)
#endif // PS1080_H
