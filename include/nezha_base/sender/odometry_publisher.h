#ifndef __EEZHA_BASE_SENDER_ODOMETRY_PUBLISHER_H__
#define __EEZHA_BASE_SENDER_ODOMETRY_PUBLISHER_H__

#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>


namespace base {

class OdometryPublisher
{
    public:
        OdometryPublisher(
            ros::NodeHandle &nh,
            std::string topic_name,
            std::string frame_id,
            std::string child_frame_id,
            int buff_size);
        void doPublish(nav_msgs::Odometry & odom, ros::Time time);
        void doPublish(const Eigen::Matrix4f& transform_matrix);
        void doPublish(const Eigen::Matrix<float,3,2>& velocity_Matrix ,const Eigen::Matrix4f & transform_matrix);

    private:
        ros::NodeHandle     m_nh;
        ros::Publisher      m_publisher;
        nav_msgs::Odometry  m_odometry;


};
}


#endif

