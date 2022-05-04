#include "../include/nve_application.h"

NveApplication::NveApplication(ros::NodeHandle *nh)
{
    // Generate nve ft processing handle
    m_nve_ft_processing = std::make_shared<NveFTProcessing>(nh);
    
}