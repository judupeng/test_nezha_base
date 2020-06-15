#ifndef __EEZHA_BASE_RECEIVER_IMU_SUBSCRIBER_H__
#define __EEZHA_BASE_RECEIVER_IMU_SUBSCRIBER_H__

#include "nezha_base/sensor_data/imu_data.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "nezha_base/sender/imu_publisher.h"
#include <deque>

namespace base {

class ImuSubscriber
{

public:
    ImuSubscriber(ros::NodeHandle &nh,std::string topic_name,int buff_size);
    void call_back(sensor_msgs::Imu & msg_corrected);
    void getData(std::deque<ImuData> &imuDataDeque);
private:
    ros::Subscriber m_imuSubscriber;
    std::shared_ptr<ImuPublisher>  m_imuPubPtr;
    std::deque<ImuData> m_imuDataDeque;
    
};

} 


#endif