#pragma once
#include <string>
#include <vector>

namespace pallet_idenfity
{
struct PalletPose {
    double x;
    double y;
    double theta;
};

class PalletIdentifyBase
{
   public:
    PalletIdentifyBase()
    {
    }
    virtual ~PalletIdentifyBase()
    {
    }
    virtual bool InitPalletIdentify(const std::string ip_camera_dtof) = 0;
    virtual bool GetPalletIdentifyData(PalletPose &pallet_identify_result) = 0;
    virtual bool TestPalletIdentifyData() = 0;

   protected:
};
}  // namespace pallet_idenfity
