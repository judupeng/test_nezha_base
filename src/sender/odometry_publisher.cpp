#include "nezha_base/sender/odometry_publisher.h"

namespace base {


OdometryPublisher::OdometryPublisher(ros::NodeHandle &nh,
            std::string topic_name,
            std::string base_frame_id,
            std::string child_frame_id,
            int buff_size)
:m_nh(nh)
{
    m_publisher= m_nh.advertise<nav_msgs::Odometry>(topic_name,buff_size);
    m_odometry.header.frame_id= base_frame_id;
    m_odometry.child_frame_id= child_frame_id;
}



void OdometryPublisher::doPublish(const Eigen::Matrix4f& transform_matrix)
{
    m_odometry.header.stamp= ros::Time::now();
    m_odometry.pose.pose.position.x= transform_matrix(0,3);
    m_odometry.pose.pose.position.y= transform_matrix(1,3);
    m_odometry.pose.pose.position.z= transform_matrix(2,3);

    Eigen::Quaternionf q;
    q= transform_matrix.block<3,3>(0,0);
    m_odometry.pose.pose.orientation.x= q.x();
    m_odometry.pose.pose.orientation.y= q.y();
    m_odometry.pose.pose.orientation.z= q.z();
    m_odometry.pose.pose.orientation.w= q.w();

    m_publisher.publish(m_odometry);

}

void OdometryPublisher::doPublish(const Eigen::Matrix<float,3,2>& velocity_Matrix ,const Eigen::Matrix4f& transform_matrix)
{

    m_odometry.header.stamp= ros::Time::now();
    m_odometry.pose.pose.position.x= transform_matrix(0,3);
    m_odometry.pose.pose.position.y= transform_matrix(1,3);
    m_odometry.pose.pose.position.z= transform_matrix(2,3);

    Eigen::Quaternionf q;
    q= transform_matrix.block<3,3>(0,0);
    m_odometry.pose.pose.orientation.x= q.x();
    m_odometry.pose.pose.orientation.y= q.y();
    m_odometry.pose.pose.orientation.z= q.z();
    m_odometry.pose.pose.orientation.w= q.w();

    m_odometry.twist.twist.linear.x= velocity_Matrix(0,0);
    m_odometry.twist.twist.linear.y= velocity_Matrix(1,0);
    m_odometry.twist.twist.linear.z= velocity_Matrix(2,0);
    m_odometry.twist.twist.angular.x= velocity_Matrix(0,1);
    m_odometry.twist.twist.angular.x= velocity_Matrix(1,1);
    m_odometry.twist.twist.angular.x= velocity_Matrix(2,1);
    m_publisher.publish(m_odometry);
}

void OdometryPublisher::doPublish(nav_msgs::Odometry & odom, ros::Time time)
{
    m_odometry.header.stamp=time;
    m_odometry.pose= odom.pose;
    m_odometry.twist= odom.twist;
    m_publisher.publish(m_odometry);
}














}
