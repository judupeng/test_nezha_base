#include "nezha_base/receiver/imu_subscriber.h"

namespace base{


ImuSubscriber::ImuSubscriber(ros::NodeHandle &nh,std::string topic_name,int buff_size)
{
	m_imuSubscriber = nh.subscribe(topic_name,buff_size,&ImuSubscriber::call_back,this);
}

void ImuSubscriber::call_back(sensor_msgs::Imu & msg_corrected)
{
	ImuData imuData;
	imuData.time= msg_corrected.header.stamp.toSec();
	imuData.linear_acceleration.x=msg_corrected.linear_acceleration.x;
	imuData.linear_acceleration.y=msg_corrected.linear_acceleration.y;
	imuData.linear_acceleration.z=msg_corrected.linear_acceleration.z;
	imuData.angular_velocity.x=msg_corrected.linear_acceleration.x;
	imuData.angular_velocity.y=msg_corrected.linear_acceleration.y;
	imuData.angular_velocity.z=msg_corrected.linear_acceleration.z;
	imuData.orientation.x = msg_corrected.orientation.x;
    imuData.orientation.y = msg_corrected.orientation.y;
    imuData.orientation.z = msg_corrected.orientation.z;
    imuData.orientation.w = msg_corrected.orientation.w;
	m_imuDataDeque.push_back(imuData);
}

void ImuSubscriber::getData(std::deque<ImuData> &imuDataDeque)
{
	if(m_imuDataDeque.size() > 0)
	{
		imuDataDeque.insert(imuDataDeque.begin(),m_imuDataDeque.begin(), m_imuDataDeque.end());
		m_imuDataDeque.clear();
	}
}

}