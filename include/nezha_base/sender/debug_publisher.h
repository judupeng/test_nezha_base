#ifndef __EEZHA_BASE_SENDER_DEBUG_PUBLISHER_H__
#define __EEZHA_BASE_SENDER_DEBUG_PUBLISHER_H__

# include <ros/ros.h>
# include <std_msgs/Int32.h>
#include "nezha_base/msg/msg_define.h"

namespace base{


class DeubgPublisher{

    public:
        DeubgPublisher(ros::NodeHandle &nh,std::string lfet_topic_name,std::string right_topic_name,int buff_size);
        DeubgPublisher(ros::NodeHandle &nh,std::string battery_topic_name,int buff_size);

        void doLeftVelPublish();
        void doRightVelPublish();
        void Publish(uint8_t* buffer_data,Encoder & encoder_data);
        void Publish(Encoder & encoder_data);
        void Publish(int left_set,int right_set);
        void Publish(int battery);


    private:

        ros::NodeHandle m_nh;
        ros::Publisher  m_leftVelPublisher;
        ros::Publisher  m_rightVelPublisher;
        ros::Publisher  m_batteryPublisher;
        uint8_t* m_bufferData;
        int m_curLeft;
        int m_curRight;
            
};





}

#endif
