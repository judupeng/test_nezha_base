#ifndef __EEZHA_BASE_SENDER_IMU_PUBLISHER_H__
#define __EEZHA_BASE_SENDER_IMU_PUBLISHER_H__


#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

namespace base {

class ImuPublisher
{
    public:
        ImuPublisher(ros::NodeHandle &nh,
            std::string topic_name,
            std::string frame_id,
            int buff_size);

        ImuPublisher(ros::NodeHandle &nh,
        std::string imu_topic_name,
        std::string mag_topic_name,
        std::string imu_frame_id,
        int buff_size);

        void doPublish(sensor_msgs::Imu & imu, ros::Time time);
        void doPublish(sensor_msgs::Imu & imu, sensor_msgs::MagneticField & mag,ros::Time time);
        void doPublish(const Eigen::Matrix4f& transform_matrix);

    private:
        ros::NodeHandle     m_nh;
        ros::Publisher      m_publisher;
        ros::Publisher      m_magPublisher;
        sensor_msgs::Imu    m_imu;
        sensor_msgs::MagneticField    m_magneticField;
        

};


}









#endif