#ifndef __EEZHA_BASE_SENDER_TF_BROADCASTER_H__
#define __EEZHA_BASE_SENDER_TF_BROADCASTER_H__

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>

namespace base {

class TfBroadCaster
{
    public:
        TfBroadCaster(std::string frame_id, std::string child_frame_id);
        void sendTransform(Eigen::Matrix4f pose, double time);
        void sendTransform(geometry_msgs::TransformStamped transformStamped, ros::Time  time);
    private:
        geometry_msgs::TransformStamped  m_transform_msgs;
        tf::StampedTransform             m_transform;
        tf::TransformBroadcaster         m_br;
};


}



#endif