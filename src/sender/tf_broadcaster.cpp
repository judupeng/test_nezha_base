#include "nezha_base/sender/tf_broadcaster.h"



namespace base
{

TfBroadCaster::TfBroadCaster(std::string frame_id, std::string child_frame_id)
{
    m_transform_msgs.header.frame_id = frame_id;
    m_transform_msgs.child_frame_id  = child_frame_id;
    m_transform.frame_id_ = frame_id;
    m_transform.child_frame_id_ = child_frame_id;
};



void TfBroadCaster::sendTransform(Eigen::Matrix4f pose, double time)
{
    Eigen::Quaternionf q(pose.block<3,3>(0,0));
    ros::Time ros_time((float)time);
    m_transform.stamp_ = ros_time;
    m_transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    m_transform.setOrigin(tf::Vector3(pose(0,3), pose(1,3), pose(2,3)));
    m_br.sendTransform(m_transform);
};


void TfBroadCaster::sendTransform(geometry_msgs::TransformStamped transformStamped, ros::Time  time)
{
    m_transform_msgs.header.stamp=time;
    m_transform_msgs.transform= transformStamped.transform;
    m_br.sendTransform(m_transform_msgs);
};

}
