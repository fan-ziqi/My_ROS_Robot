#ifndef  __UTIL_H__
#define __UTIL_H__


#ifndef true
#define true 1
#endif
#ifndef false
#define false 0
#endif
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
//typedef int bool;
typedef bool BOOL;
typedef long LONG;
typedef unsigned long ULONG;
typedef unsigned char BYTE;
typedef unsigned short USHORT;
typedef unsigned int DWORD;

#define SAFE_DELETE_ARRAY(x) { if (x) free(x); x = NULL; }

typedef enum
{
	EC_UnKnow,
	EC_EnumDeviceFail,
	EC_DisableFlashWriteProtectFail,
	EC_EraseFlashFail,
	EC_EraseFlashSectorFail,
	EC_GetAsicIdFail,
	EC_GetAsicRomVersionFail,
	EC_GetAsicRomTypeFail,
	EC_ReadAsicRegisterFail,
	EC_WriteAsicRegisterFail,
	EC_UnKnowSerialFlashType,
	EC_BurnerCheckFail,
	EC_CoInitializeFail,
	EC_NoFindDevice,
	EC_MallocMemoryFail,
}ERROR_CODE;


typedef enum
{
	SFT_UNKNOW,
	SFT_MXIC,
	SFT_ST,
	SFT_SST,
	SFT_ATMEL_AT25F,
	SFT_ATMEL_AT25FS,
	SFT_ATMEL_AT45DB,
	SFT_WINBOND,
	SFT_PMC,
	SFT_MXIC_LIKE	,
	SFT_AMIC,
	SFT_EON,
	SF_ESMT,
	SFT_GIGA,
	SFT_FENTECH
}SERIAL_FLASH_TYPE;

typedef enum
{
	DRT_Unknow,
	DRT_32K,
	DRT_64K,
	DRT_128K,
	DRT_256K
}DSP_ROM_TYPE;
#endif

