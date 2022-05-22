#include "astra_camera/astra_device_type.h"

bool astraWithUVC(OB_DEVICE_NO id)
{
    if (id == OB_STEREO_S_NO || id == OB_EMBEDDED_S_NO || id == OB_ASTRA_PRO_NO || id == OB_STEREO_S_U3_NO)
        return true;
    return false;
}