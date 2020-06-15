#ifndef __EEZHA_BASE_SENDER_SONAR_PUBLISHER_H__
#define __EEZHA_BASE_SENDER_SONAR_PUBLISHER_H__
#include <sensor_msgs/Range.h>
#include <ros/ros.h>

namespace base{

class SonarPublisher{

    public:
        SonarPublisher(ros::NodeHandle & nh,std::string topic_name,int buf_size);
        void doPublish(std::vector<sensor_msgs::Range> sonar_vector,int port);
    public:
        ros::Publisher m_sonarPublisher;
        ros::NodeHandle m_nh;
        std::string m_topicName;
        int m_bufSize;
        int m_sonarNum;

};

}


#endif

