#ifndef ROSUNIT_EMPTYCLNT
#define ROSUNIT_EMPTYCLNT

#include <ros/ros.h>
#include <std_srvs/Empty.h>

namespace HEAR{

class ROSUnitEmptyClient {
private:
    ros::ServiceClient m_client;
public:
    ROSUnitEmptyClient(ros::NodeHandle&, std::string);
    bool process();
};

}

#endif