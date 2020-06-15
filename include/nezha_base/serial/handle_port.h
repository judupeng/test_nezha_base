# ifndef __EEZHA_BASE_SERIAL_HANDLE_PORT_H__
# define __EEZHA_BASE_SERIAL_HANDLE_PORT_H__

#include <geometry_msgs/TransformStamped.h>
# include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include "nezha_base/msg/msg_define.h"
#include "nezha_base/sensor_data/imu_data.h"
#include "nezha_base/sender/odometry_publisher.h"
#include "nezha_base/sender/imu_publisher.h"
#include "nezha_base/sender/debug_publisher.h"

namespace base{


class HandlePort{

    public:
        HandlePort();
        void handleWheelData(Encoder & encoder_data);
        void handleWheelDataDirect(Encoder & encoder_data);
        void mergeWheelData(uint8_t* buffer_left,uint8_t* buffer_right,Encoder & encoder_data);
        void handleImuData(uint8_t* buffer_data,sensor_msgs::Imu imuCalibData );
        void handleBatteryData(uint8_t* buffer_data);
        void handleSonarData(uint8_t* buffer_data,int port_id);
        static void checkSum(uint8_t* data, size_t len, uint8_t& dest);

    public:
        geometry_msgs::TransformStamped  m_odomTransform;
        std::shared_ptr<ImuData>      m_imuDataPtr;
        nav_msgs::Odometry            m_odomMsg;
        sensor_msgs::Imu              m_imuMsg;
        sensor_msgs::MagneticField    m_magMsg;

        std_msgs::Float32   m_batteryMsg;
        sensor_msgs::Range  m_sonarMsg;

        std::vector<sensor_msgs::Range> m_sonarVector;
        double ax_roll_=0;
        double ax_pitch_=0;
        double ax_yaw_=0;
        int    imu_msg_count_=0;

};




}



#endif