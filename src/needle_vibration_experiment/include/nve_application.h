#pragma once

#include <iostream>
#include <memory>
#include "ros/ros.h"
#include "nve_ft_processing.h"

class NveApplication
{
public:
    NveApplication(ros::NodeHandle *nh);

private:

   // Ft data processing handle
    std::shared_ptr<NveFTProcessing> m_nve_ft_processing;

};



