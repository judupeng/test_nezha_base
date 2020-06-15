#include "nezha_base/sender/debug_publisher.h"


namespace base{

DeubgPublisher::DeubgPublisher(ros::NodeHandle &nh,std::string lfet_topic_name,std::string right_topic_name,int buff_size )
{
    m_nh=nh;
    m_leftVelPublisher=m_nh.advertise<std_msgs::Int32>(lfet_topic_name,buff_size);
    m_rightVelPublisher=m_nh.advertise<std_msgs::Int32>(right_topic_name,buff_size);
};

DeubgPublisher::DeubgPublisher(ros::NodeHandle &nh,std::string battery_topic_name,int buff_size )
{
    m_nh=nh;
    m_batteryPublisher=m_nh.advertise<std_msgs::Int32>(battery_topic_name,buff_size);
};

void DeubgPublisher::doLeftVelPublish()
{
    std_msgs::Int32 lvel_data;
    lvel_data.data = (m_bufferData[4]*256 + m_bufferData[5]) - m_curLeft;
    m_leftVelPublisher.publish(lvel_data);
};

void DeubgPublisher::doRightVelPublish()
{
    std_msgs::Int32 rvel_data;
    rvel_data.data = m_bufferData[6]*256 + m_bufferData[7] - m_curRight;

    m_rightVelPublisher.publish(rvel_data);
};

void DeubgPublisher::Publish(uint8_t* buffer_data, Encoder & encoder_data)
{
    m_bufferData=buffer_data;
    
    std_msgs::Int32 lvel_data,rvel_data;
    int rec_left,rec_right;
    rec_left=m_bufferData[4]*256 + m_bufferData[5];
    rec_right=m_bufferData[6]*256 + m_bufferData[7];
    lvel_data.data = rec_left - encoder_data.cur_left_;
    rvel_data.data = rec_right - encoder_data.cur_right_;
    
    m_leftVelPublisher.publish(lvel_data);
    m_rightVelPublisher.publish(rvel_data);

    encoder_data.rev_left_=rec_left;
    encoder_data.rev_right_=rec_right;
};

//  用于哪吒两条腿
void DeubgPublisher::Publish(Encoder & encoder_data)
{
    std_msgs::Int32 lvel_data,rvel_data;
    lvel_data.data = encoder_data.speed_rpm_left_;
    rvel_data.data = encoder_data.speed_rpm_right_;
    m_leftVelPublisher.publish(lvel_data);
    m_rightVelPublisher.publish(rvel_data);
}

void DeubgPublisher::Publish(int left_set,int right_set)
{
    std_msgs::Int32 ldata_set,rdata_set;
    ldata_set.data=left_set;
    rdata_set.data=right_set;
    m_leftVelPublisher.publish(ldata_set);
    m_rightVelPublisher.publish(rdata_set);
};

void DeubgPublisher::Publish(int battery)
{
    std_msgs::Int32 batteryInfo;
    batteryInfo.data=battery;
    m_batteryPublisher.publish(batteryInfo);
};


    
}

