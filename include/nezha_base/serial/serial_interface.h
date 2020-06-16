# ifndef __EEZHA_BASE_SERIAL_SERIAL_INTERFACE_H__
# define __EEZHA_BASE_SERIAL_SERIAL_INTERFACE_H__

#include <iostream>  
#include <string>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include "../../../../devel/include/test_nezha_base/pid_configConfig.h"
#include "nezha_base/sender/odometry_publisher.h"
#include "nezha_base/sender/imu_publisher.h"
#include "nezha_base/sender/debug_publisher.h"
#include "nezha_base/msg/msg_define.h"
#include "nezha_base/sender/tf_broadcaster.h"
#include "nezha_base/serial/handle_port.h"
#include "nezha_base/sender/sonar_publisher.h"
#include "nezha_base/msg/ServoMsg.h"

namespace base {

typedef boost::shared_ptr<boost::asio::serial_port> Serial_ptr;

class SerialInterface
{

    public:
        SerialInterface();
        ~SerialInterface();

    public:
        void loop();
        bool initPort(Serial_ptr &serialPtr,std::string portName,
                boost::asio::io_service *ioService,boost::system::error_code & errorCode);
        void runParsing(Serial_ptr &serialPtr,boost::system::error_code &errorCode,int port_id);
        void receveParsing();
        void receveParsingLeft();
        void receveParsingRight();
        void parsingData(uint8_t port_msgType, uint8_t* _buffer,int port_id);
        //void checkSum(uint8_t* data, size_t len, uint8_t& dest);
        void cmdVelCallback(const geometry_msgs::Twist & twist);
        void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void servo_msg_callback(const servo_node::ServoMsg& msg);

        void timeEventCallback(const ros::TimerEvent&);
        void dynamic_reconfig_callback(nezha_base::pid_configConfig &config);
        void sendLeftSpeed();
        void sendRightSpeed();
        double steer(double steer_angle);
        double mapDouble(double x,double in_min,double in_max,double out_min,double out_max);
        double constrain(double data,double min_limit, double max_limit);
        void getCopy(uint8_t* _buffer,uint8_t *m_bufGet,int length);

        static void handler(const boost::system::error_code& error, std::size_t bytes_transferred);

    private:
        
        std::shared_ptr<OdometryPublisher> m_odomPubPtr;
        std::shared_ptr<ImuPublisher>      m_imuPubPtr;
        std::shared_ptr<DeubgPublisher>    m_lrTicksPubPtr;
        std::shared_ptr<DeubgPublisher>    m_lrTicksSetPubPtr;
        std::shared_ptr<DeubgPublisher>    m_battertPubPtr;
        std::shared_ptr<SonarPublisher>    m_sonarPubPtr;
        
        std::shared_ptr<TfBroadCaster>     m_tfBroadcasePtr;

        std::shared_ptr<HandlePort> m_handlePortptr;

        Serial_ptr m_serialPtr;
        Serial_ptr m_serialLeftPtr;
        Serial_ptr m_serialRigthPtr;

        std::string m_portName;
        std::string m_leftPortName;
        std::string m_rightPortName;

        boost::asio::io_service m_ioService; 
        boost::asio::io_service m_ioServiceLeft;
        boost::asio::io_service m_ioServiceRight; 

        boost::system::error_code m_errorCode; 
        boost::system::error_code m_errorCodeLeft; 
        boost::system::error_code m_errorCodeRight;   

        boost::condition_variable m_condvar;
        boost::mutex m_mutex;
        boost::mutex m_mutexLeft;
        boost::mutex m_mutexRight;
        boost::mutex m_mutexSend;

        Encoder m_encoderData;
        geometry_msgs::Twist m_twistRaw;
        sensor_msgs::Imu m_imuCalibData;

        ros::Subscriber   m_servoMsgSub;
        ros::Subscriber   m_cmdSub;
        ros::Subscriber   m_imuCalibSub;
        ros::Timer m_sendVelBack;
        ros::Time m_lastTwistTime;

        std::string odom_frame_;
        std::string imu_frame_;
	    std::string lidar_frame_;
        std::string base_frame_;

        int m_firstInit;
        int m_baudRate;
        int m_controlRate;
        int m_sensor_rate;
        int m_leftTicksSet;
        int m_rightTicksSet;
        unsigned char m_bufLeftSend[15];
        unsigned char m_bufRightSend[15];
        unsigned char m_bufSteeringAngleSend[15];
        uint8_t m_bufLeftGet[15];
        uint8_t m_bufRightGet[15];
        bool LEFT_GET,RIGHT_GET;
        bool LETF_SEND,RTGHT_SEND;
        bool is_publish_odom_transform;
};

}


#endif
