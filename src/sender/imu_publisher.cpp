#include "nezha_base/sender/imu_publisher.h"

namespace base {

    ImuPublisher::ImuPublisher(ros::NodeHandle &nh,
        std::string topic_name,
        std::string frame_id,
        int buff_size)
    {
        m_nh=nh;
        m_imu.header.frame_id= frame_id;
        m_publisher= m_nh.advertise<sensor_msgs::Imu>(topic_name,buff_size);
    }

    ImuPublisher::ImuPublisher(ros::NodeHandle &nh,
        std::string imu_topic_name,
        std::string mag_topic_name,
        std::string imu_frame_id,
        int buff_size)
    {
        m_nh=nh;
        m_imu.header.frame_id= imu_frame_id;
        m_magneticField.header.frame_id= imu_frame_id;
        m_publisher= m_nh.advertise<sensor_msgs::Imu>(imu_topic_name,buff_size);
        m_magPublisher= m_nh.advertise<sensor_msgs::MagneticField>(mag_topic_name,buff_size);
    }

    void ImuPublisher::doPublish(sensor_msgs::Imu & imu, ros::Time time)
    {
        m_imu.header.stamp=time;
        m_imu.orientation=imu.orientation;
        m_imu.orientation_covariance=imu.orientation_covariance;
        m_imu.angular_velocity=imu.angular_velocity;
        m_imu.angular_velocity_covariance=imu.angular_velocity_covariance;
        m_imu.linear_acceleration=imu.linear_acceleration;
        m_imu.linear_acceleration_covariance=imu.linear_acceleration_covariance;
        m_publisher.publish(m_imu);
    }

    void ImuPublisher::doPublish(sensor_msgs::Imu & imu, sensor_msgs::MagneticField & mag,ros::Time time)
    {
        m_imu.header.stamp=time;
        m_imu.orientation=imu.orientation;
        m_imu.orientation_covariance=imu.orientation_covariance;
        m_imu.angular_velocity=imu.angular_velocity;
        m_imu.angular_velocity_covariance=imu.angular_velocity_covariance;
        m_imu.linear_acceleration=imu.linear_acceleration;
        m_imu.linear_acceleration_covariance=imu.linear_acceleration_covariance;
        m_publisher.publish(m_imu);

        m_magneticField.header.stamp=time;
        m_magneticField.magnetic_field=mag.magnetic_field;
        m_magPublisher.publish(m_magneticField);
    }

}