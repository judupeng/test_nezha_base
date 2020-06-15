#include "nezha_base/sender/sonar_publisher.h"

namespace base
{

    SonarPublisher::SonarPublisher(ros::NodeHandle & nh,std::string topic_name,int buf_size)
    {
        m_nh=nh;
        m_topicName=topic_name;
        m_bufSize=buf_size;
    };

    void SonarPublisher::doPublish(std::vector<sensor_msgs::Range> sonar_vector,int port)
    {
        for (int i=0; i<2; i++ )
        {
            m_sonarPublisher=m_nh.advertise<sensor_msgs::Range>(m_topicName+"_"+std::to_string(i+2*port),m_bufSize);
            m_sonarPublisher.publish(sonar_vector[i]);
        }
    };

}