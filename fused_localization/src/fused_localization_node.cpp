#include "fused_localization/fused_localization.h"
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fused_localization_node");
    std::unique_ptr<fused_localization::FusedLocalization> fused_localization_prt = 
                                            std::make_unique<fused_localization::FusedLocalization>();
    bool init_flag = fused_localization_prt->Init();
    ros::spin();

    return 0;
}
