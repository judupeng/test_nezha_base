#include "nezha_base/serial/serial_interface.h"
#include "nezha_base/serial/handle_port.h"


namespace base {

SerialInterface::SerialInterface() {}

void SerialInterface::loop()
{
    ros::NodeHandle nh;
	ros::NodeHandle nh_p("~");	
    m_firstInit=0;
    LETF_SEND=0;
    RTGHT_SEND=0;
    LEFT_GET=0;
    RIGHT_GET=0;
    nh_p.param<std::string>("base_frame",base_frame_,std::string("base_footprint"));
    nh_p.param<std::string>("imu_frame",imu_frame_,std::string("imu_link"));
    nh_p.param<std::string>("lidar_frame",lidar_frame_,std::string("laser"));
    nh_p.param<std::string>("left_port_name",m_leftPortName,std::string("/dev/ttyUART0"));
    nh_p.param<std::string>("right_port_name",m_rightPortName,std::string("/dev/ttyUART1"));
    nh_p.param<std::string>("port_name",m_portName,std::string("/dev/ttyS4"));                  
    nh_p.param<int>("baud_rate",m_baudRate,115200);
    nh_p.param<double>("encoder_resolution",m_encoderData.encoder_resolution_,8192);
    nh_p.param<double>("wheel_diameter",m_encoderData.wheel_diameter_,0.095);
    nh_p.param<double>("wheel_track_fb",m_encoderData.wheel_track_,0.20);
    nh_p.param<double>("wheel_track_rl",m_encoderData.wheel_distance_,0.352);
    nh_p.param<double>("pid_rate",m_encoderData.pid_rate_,100);    
    nh_p.param<double>("gear_reduction",m_encoderData.gear_reduction_,36);
    nh_p.param<int>("encoder_min",m_encoderData.encoder_min_,-2147483648);// int16_t: 2^16=32768  int32_t  2^31= 2147483648
    nh_p.param<int>("encoder_max",m_encoderData.encoder_max_,2147483648);
    nh_p.param<double>("encoder_low_wrap",m_encoderData.encoder_low_wrap_,(m_encoderData.encoder_max_-m_encoderData.encoder_min_)*0.3 + m_encoderData.encoder_min_);
    nh_p.param<double>("encoder_high_wrap",m_encoderData.encoder_high_wrap_,(m_encoderData.encoder_max_-m_encoderData.encoder_min_)*0.7 + m_encoderData.encoder_min_);
    nh_p.param<bool>("publish_odom_transform",is_publish_odom_transform,false);
    nh_p.param<int>("Kp",m_encoderData.kp_,1500); // 0.5  500
    nh_p.param<int>("Ki",m_encoderData.ki_,100);// 0.05    50
    nh_p.param<int>("Kd",m_encoderData.kd_,0);//0.3      300
    nh_p.param<double>("accel_limit",m_encoderData.accel_limit_,0.1);
    nh_p.param<double>("max_linear_speed",m_encoderData.max_linear_speed_,0.5);
    nh_p.param<int>("control_rate",m_controlRate,100);
    nh_p.param<double>("linear_correction_factor",m_encoderData.linear_correction_factor_,1.0);
    nh_p.param<double>("max_steering_angle",m_encoderData.max_steering_angle,15);
    nh_p.param<double>("servo_bias",m_encoderData.servo_bias_,1900);
    nh_p.param<double>("steering_offset_factor",m_encoderData.steering_offset_factor,1.0);

    m_encoderData.max_linear_speed_= (m_encoderData.max_linear_speed_ > 2.0)? 2.0 : m_encoderData.max_linear_speed_;
    m_encoderData.max_linear_speed_= (m_encoderData.max_linear_speed_ < -2.0)? -2.0 : m_encoderData.max_linear_speed_;
    // Differential_Drive  Four_wheeled_Ackerman
    m_encoderData.carMode=Four_wheeled_Ackerman;
    m_encoderData.set_servo_steering_angle= m_encoderData.servo_bias_;
    m_encoderData.max_steering_angle = m_encoderData.max_steering_angle/180*M_PI;
    m_encoderData.encoder_resolution_ = m_encoderData.encoder_resolution_/m_encoderData.linear_correction_factor_;
    m_encoderData.ticks_per_meter_ = m_encoderData.encoder_resolution_ * m_encoderData.gear_reduction_/(m_encoderData.wheel_diameter_ * M_PI);
    m_encoderData.cur_left_=0;
    m_encoderData.cur_right_=0;
    m_encoderData.start_flag_=true;

    m_handlePortptr = std::make_shared<HandlePort>();
    m_lastTwistTime=ros::Time::now();
    m_lastTwistTime.sec=m_lastTwistTime.sec-1;
    m_twistRaw= geometry_msgs::Twist();

    m_odomPubPtr = std::make_shared<OdometryPublisher>(nh, "/odom", odom_frame_, base_frame_, 100);
    m_imuPubPtr  = std::make_shared<ImuPublisher>(nh, "/imu_raw","/mag_raw",imu_frame_,100);
    m_lrTicksPubPtr =std::make_shared<DeubgPublisher>(nh,"/nezha/ltickets","/nezha/rtickets",10);
    m_lrTicksSetPubPtr= std::make_shared<DeubgPublisher>(nh,"/nezha/lticketsSet","/nezha/rticketsSet",10);
    m_battertPubPtr= std::make_shared<DeubgPublisher>(nh,"/voltage",1);
    m_sonarPubPtr=std::make_shared<SonarPublisher>(nh,"/ultrasound",5);
    m_tfBroadcasePtr = std::make_shared<TfBroadCaster>(odom_frame_,base_frame_);
    m_cmdSub = nh.subscribe("/cmd_vel",10,&SerialInterface::cmdVelCallback,this);
    m_imuCalibSub= nh.subscribe("/corrected",10,&SerialInterface::imu_callback,this);
    m_servoMsgSub = nh.subscribe("/servo_msg",10,&SerialInterface::servo_msg_callback,this);

    m_sendVelBack = nh.createTimer(ros::Duration(1.0/m_controlRate),&SerialInterface::timeEventCallback,this);

    dynamic_reconfigure::Server<nezha_base::pid_configConfig> reconfig_server;
	dynamic_reconfigure::Server<nezha_base::pid_configConfig>::CallbackType f;
	f = boost::bind(&SerialInterface::dynamic_reconfig_callback,this,_1);
	reconfig_server.setCallback(f);
       
    if (initPort(m_serialLeftPtr,m_leftPortName,&m_ioServiceLeft,m_errorCodeLeft) && 
            initPort(m_serialRigthPtr,m_rightPortName,&m_ioServiceRight,m_errorCodeRight)) 
    {
        ROS_INFO("initPort /dev/ttyUART0");
        ROS_INFO("initPort /dev/ttyUART1");	
        boost::thread thread_left_send(boost::bind(&SerialInterface::sendLeftSpeed,this));
        boost::thread thread_right_send(boost::bind(&SerialInterface::sendRightSpeed,this));
        boost::thread thread_left_receve(boost::bind(&SerialInterface::receveParsingLeft,this));
        boost::thread thread_right_receve(boost::bind(&SerialInterface::receveParsingRight,this));
    }

    if (initPort(m_serialPtr,m_portName,&m_ioService,m_errorCode))
    {
        ROS_INFO("initPort /dev/ttyS4");
        boost::thread recv_thread(boost::bind(&SerialInterface::receveParsing,this));
    }
    ros::spin();

}

SerialInterface::~SerialInterface()
{
    boost::mutex::scoped_lock look(m_mutex);
    ROS_INFO("leave left rigth  port");
    if(m_serialLeftPtr)
    {
        m_serialLeftPtr->cancel();
        m_serialLeftPtr->close();
        m_serialLeftPtr.reset();
    }
    if(m_serialRigthPtr)
    {
        m_serialRigthPtr->cancel();
        m_serialRigthPtr->close();
        m_serialRigthPtr.reset();
    }
    if(m_serialPtr)
    {
        m_serialPtr->cancel();
        m_serialPtr->close();
        m_serialPtr.reset();
    }
    m_ioServiceLeft.stop();
    m_ioServiceLeft.reset();
    m_ioServiceRight.stop();
    m_ioServiceRight.reset();
    m_ioService.stop();
    m_ioService.reset();

}

bool SerialInterface::initPort(Serial_ptr &serialPtr,std::string portName,
        boost::asio::io_service *ioService,boost::system::error_code &errorCode)
{
    if(serialPtr)
    {
        std::cout<< "The SerialPort is already opened!" << std::endl;
        return false;
    }
    serialPtr=Serial_ptr(new boost::asio::serial_port(*ioService));
    serialPtr->open(portName,errorCode);
    if (errorCode)
    {
        std::cout<< "error : port_->open() failed...port_name=" << portName
            <<", e=" << errorCode.message().c_str() << std::endl;
        return false;
    }
    serialPtr->set_option(boost::asio::serial_port_base::baud_rate(m_baudRate));
    serialPtr->set_option(boost::asio::serial_port_base::character_size(8));
    serialPtr->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serialPtr->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serialPtr->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    short p_send_ = (short)m_encoderData.kp_;
    short i_send_ = (short)m_encoderData.ki_;
    short d_send_ = (short)m_encoderData.kd_;
    uint8_t data_stop[9];
    uint8_t data[11];
    data_stop[0] = HEAD_BYTE1;
    data_stop[1] = HEAD_BYTE2;
    data_stop[2] = 0x0b;
    data_stop[3] = SEND_SPEED;
    data_stop[4] = 0x00;
    data_stop[5] = 0x00;
    data_stop[6] = 0x00;
    data_stop[7] = 0x00;
    HandlePort::checkSum(data_stop,8,data_stop[8]);
    boost::asio::write(*serialPtr.get(),boost::asio::buffer(data_stop,9),errorCode);

    data[0] = HEAD_BYTE1;
    data[1] = HEAD_BYTE2;
    data[2] = 0x0b;
    data[3] = SEND_PID;
    data[4] = (p_send_ >> 8) & 0xff;
    data[5] = p_send_ & 0xff;
    data[6] = (i_send_ >> 8) & 0xff;
    data[7] = i_send_ & 0xff;
    data[8] = (d_send_ >> 8) & 0xff;
    data[9] = d_send_ & 0xff;
    HandlePort::checkSum(data,10,data[10]);

    boost::asio::write(*serialPtr.get(),boost::asio::buffer(data,11),errorCode);


    m_bufSteeringAngleSend[0] = HEAD_BYTE1;
    m_bufSteeringAngleSend[1] = HEAD_BYTE2;
    m_bufSteeringAngleSend[2] = 0x07;
    m_bufSteeringAngleSend[3] = SEND_STEER;
    m_bufSteeringAngleSend[4] = ((short)m_encoderData.set_servo_steering_angle>>8) & 0xff;
    m_bufSteeringAngleSend[5] = (short)m_encoderData.set_servo_steering_angle & 0xff;
    HandlePort::checkSum(m_bufSteeringAngleSend,6,m_bufSteeringAngleSend[6]);
    boost::asio::write(*serialPtr.get(),boost::asio::buffer(m_bufSteeringAngleSend,7),errorCode);
    return true;
   
}

void SerialInterface::dynamic_reconfig_callback(nezha_base::pid_configConfig &config)
{
   if(m_firstInit)
   {
    uint8_t pid_set[11];
    pid_set[0] = HEAD_BYTE1;
    pid_set[1] = HEAD_BYTE2;
    pid_set[2] = 0x0b;
    pid_set[3] = SEND_PID;
    pid_set[4] = (config.Kp >> 8) & 0xff;
    pid_set[5] = config.Kp & 0xff;
    pid_set[6] = (config.Ki >> 8) & 0xff;
    pid_set[7] = config.Ki & 0xff;
    pid_set[8] = (config.Kd >> 8) & 0xff;
    pid_set[9] = config.Kd & 0xff;

    m_encoderData.servo_bias_ = config.ServoBias;    // 1500  1900  2300    0 4000
    m_encoderData.max_steering_angle = config.MaxSteeringAngle/180*M_PI;   
    m_encoderData.steering_offset_factor = config.SteeringOffsetFactor;

    HandlePort::checkSum(pid_set,10,pid_set[10]);
    try{
    m_mutexLeft.lock();
    boost::asio::write(*m_serialLeftPtr.get(),boost::asio::buffer(pid_set,11),m_errorCodeLeft);
    m_mutexLeft.unlock();
    m_mutexRight.lock();
    boost::asio::write(*m_serialRigthPtr.get(),boost::asio::buffer(pid_set,11),m_errorCodeRight);
    m_mutexRight.unlock();
    }
    catch (boost::system::system_error &e)  {}
   }
   else
   {
       m_firstInit=1;
   }
}

void SerialInterface::getCopy(uint8_t* _buffer,uint8_t *m_bufGet,int length)
{
    for (int i=0;i< length; i++)
    {
        m_bufGet[i]=_buffer[i];
    }
}

void SerialInterface::cmdVelCallback(const geometry_msgs::Twist & twist)
{
    m_lastTwistTime=ros::Time::now();
    m_twistRaw= twist;
}

void SerialInterface::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    m_imuCalibData=*msg;
}


void SerialInterface::servo_msg_callback(const servo_node::ServoMsg& msg)
{
    servo_node::ServoMsg  servo_msg=msg;
    uint8_t buf_set[10];
    buf_set[0] = HEAD_BYTE1;
    buf_set[1] = HEAD_BYTE2;
    buf_set[2] = 0x0A;
    buf_set[3] = SERVMSG;
    buf_set[4] = (servo_msg.ID) & 0xff;
    buf_set[5] = (servo_msg.position >> 8) & 0xff;
    buf_set[6] = servo_msg.position & 0xff;
    buf_set[7] = (servo_msg.TIME >> 8) & 0xff;
    buf_set[8] = servo_msg.TIME & 0xff;
    HandlePort::checkSum(buf_set,9,buf_set[9]);
    std::cout << "servo_msg.ID =" << servo_msg.ID << std::endl;
    std::cout << "servo_msg.position =" << servo_msg.position << std::endl;
    std::cout << "servo_msg.TIME =" << servo_msg.TIME << std::endl;
    
    if(servo_msg.ID <= 20){
         boost::asio::write(*m_serialPtr.get(),boost::asio::buffer(buf_set,10),m_errorCode);
    }
	else if(servo_msg.ID <= 26)
    {
         boost::asio::write(*m_serialLeftPtr.get(),boost::asio::buffer(buf_set,10),m_errorCodeLeft);
    }

	else if(servo_msg.ID <= 32)
    {
        boost::asio::write(*m_serialRigthPtr.get(),boost::asio::buffer(buf_set,10),m_errorCodeRight);
    }
}




void SerialInterface::timeEventCallback(const ros::TimerEvent&)
{
    double linearSpeed, angularSpeed, rightSpeed, leftSpeed;
    double _rightSpeed, _leftSpeed;
    short  rghtTicksSet, leftTicksSet;

    if((ros::Time::now()-m_lastTwistTime).toSec()<=1)
    {
        linearSpeed= m_twistRaw.linear.x;  
        angularSpeed= m_twistRaw.angular.z;
        if(angularSpeed > 0)
		    angularSpeed = angularSpeed*m_encoderData.steering_offset_factor;
    }
    else
    {
        linearSpeed=0;
        angularSpeed=0;
    }
    
    linearSpeed= (linearSpeed > m_encoderData.max_linear_speed_)? m_encoderData.max_linear_speed_:linearSpeed;
    linearSpeed= (linearSpeed < -m_encoderData.max_linear_speed_)? -m_encoderData.max_linear_speed_:linearSpeed;

    switch (m_encoderData.carMode)
    {
        case Differential_Drive:
        {
            leftSpeed= linearSpeed - 0.5*angularSpeed * m_encoderData.wheel_distance_;
            rightSpeed= linearSpeed + 0.5*angularSpeed * m_encoderData.wheel_distance_;
            break;
        }
        case Four_wheeled_Ackerman:
        {   
            
            m_encoderData.current_steering_angle_ = steer(angularSpeed);
            
            //  right单位是米,1秒走过的距离
            leftSpeed  = linearSpeed*(1-m_encoderData.wheel_distance_*tan(m_encoderData.current_steering_angle_)/2/m_encoderData.wheel_track_);
            rightSpeed = linearSpeed*(1+m_encoderData.wheel_distance_*tan(m_encoderData.current_steering_angle_)/2/m_encoderData.wheel_track_);
            break;
        }
    }
    // 单位电机：转/分钟
    _leftSpeed=leftSpeed*60*m_encoderData.gear_reduction_/(M_PI*m_encoderData.wheel_diameter_);// 单位电机：转/分钟
    _rightSpeed=-rightSpeed*60*m_encoderData.gear_reduction_/(M_PI*m_encoderData.wheel_diameter_); 
    m_leftTicksSet=round(_leftSpeed);
    m_rightTicksSet=round(_rightSpeed);
    m_lrTicksSetPubPtr->Publish(m_leftTicksSet,m_rightTicksSet);
    
    m_bufSteeringAngleSend[0] = HEAD_BYTE1;
    m_bufSteeringAngleSend[1] = HEAD_BYTE2;
    m_bufSteeringAngleSend[2] = 0x07;
    m_bufSteeringAngleSend[3] = SEND_STEER;
    m_bufSteeringAngleSend[4] = ((short)m_encoderData.set_servo_steering_angle>>8) & 0xff;
    m_bufSteeringAngleSend[5] = (short)m_encoderData.set_servo_steering_angle & 0xff;
    HandlePort::checkSum(m_bufSteeringAngleSend,6,m_bufSteeringAngleSend[6]);
    try {
    boost::asio::write(*m_serialPtr.get(),boost::asio::buffer(m_bufSteeringAngleSend,7),m_errorCode);
    }
    catch (boost::system::system_error &e)  {}
    //printf("\n SET servo_steering_angle = %d: \n",(short)m_encoderData.set_servo_steering_angle);
    LETF_SEND=1;
    RTGHT_SEND=1; 
 }

void SerialInterface::sendLeftSpeed()
{
    while(1)
   {
     if (LETF_SEND)
     {
        try{
        m_mutexLeft.lock();

        boost::asio::write(*m_serialLeftPtr.get(),boost::asio::buffer(m_bufSteeringAngleSend,7),m_errorCodeLeft);


        LETF_SEND=0;
        m_bufLeftSend[0] = HEAD_BYTE1;
        m_bufLeftSend[1] = HEAD_BYTE2;
        m_bufLeftSend[2] = 0x0b;
        m_bufLeftSend[3] = SEND_SPEED;
        m_bufLeftSend[4] = (m_leftTicksSet >> 24) & 0xff;
        m_bufLeftSend[5] = (m_leftTicksSet >> 16) & 0xff;
        m_bufLeftSend[6] = (m_leftTicksSet >> 8) & 0xff;
        m_bufLeftSend[7] = m_leftTicksSet & 0xff;
        HandlePort::checkSum(m_bufLeftSend,8,m_bufLeftSend[8]);
        boost::asio::write(*m_serialLeftPtr.get(),boost::asio::buffer(m_bufLeftSend,9),m_errorCodeLeft);
        //printf("\n SET left = %d: \n",m_leftTicksSet);
        m_mutexLeft.unlock();
        }
        catch (boost::system::system_error &e)  {}
     }
    }
}

void SerialInterface::sendRightSpeed()
{

    while(1)
    {
	  if (RTGHT_SEND)
	  {
        try{
        m_mutexRight.lock();
        RTGHT_SEND=0;
        boost::asio::write(*m_serialRigthPtr.get(),boost::asio::buffer(m_bufSteeringAngleSend,7),m_errorCodeRight);

        m_bufRightSend[0] = HEAD_BYTE1;
        m_bufRightSend[1] = HEAD_BYTE2;
        m_bufRightSend[2] = 0x09;
        m_bufRightSend[3] = SEND_SPEED;
        m_bufRightSend[4] = (m_rightTicksSet >> 24) & 0xff;
        m_bufRightSend[5] = (m_rightTicksSet >> 16) & 0xff;
        m_bufRightSend[6] = (m_rightTicksSet >> 8) & 0xff;
        m_bufRightSend[7] = m_rightTicksSet & 0xff;
        HandlePort::checkSum(m_bufRightSend,8,m_bufRightSend[8]);
        boost::asio::write(*m_serialRigthPtr.get(),boost::asio::buffer(m_bufRightSend,9),m_errorCodeRight);
        //printf("\n SET right = %d: \n",m_rightTicksSet);
        m_mutexRight.unlock();
        }
        catch (boost::system::system_error &e)  {}
      } 
    }
}

void SerialInterface::receveParsing()
{
    runParsing(m_serialPtr,m_errorCode,0);
}

void SerialInterface::receveParsingLeft()
{
    runParsing(m_serialLeftPtr,m_errorCodeLeft,1);
}

void SerialInterface::receveParsingRight()
{
    runParsing(m_serialRigthPtr,m_errorCodeRight,2);
}


double SerialInterface::constrain(double data,double min_limit, double max_limit)
{
    if(data>max_limit)
        return max_limit;
    if(data < min_limit)
        return min_limit;
    return data;
}
double SerialInterface::mapDouble(double x,double in_min,double in_max,double out_min,double out_max)
{
    return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
double SerialInterface::steer(double steer_angle)
{
    double max_steering_angle_= m_encoderData.max_steering_angle;
    steer_angle = constrain(steer_angle,-max_steering_angle_,max_steering_angle_);                                                                    // (180/M_PI)*11.111111=636.62031 
    m_encoderData.set_servo_steering_angle = m_encoderData.servo_bias_-mapDouble(steer_angle,-max_steering_angle_,max_steering_angle_,-max_steering_angle_,max_steering_angle_)*636.62031 ;
    if(m_encoderData.set_servo_steering_angle > 2300)
        m_encoderData.set_servo_steering_angle=2300;
    if(m_encoderData.set_servo_steering_angle < 1500)
        m_encoderData.set_servo_steering_angle=1500;
    return steer_angle;
}

void SerialInterface::runParsing(Serial_ptr &serialPtr,boost::system::error_code &errorCode,int port_id)
{
    uint8_t _size, check_num, _buffer[255],port_msgType,buf_tmp[2];
    ParsingState parsingState = WATI_BYTES0;
    bool recv_flag_ = true;

      try{
        while(recv_flag_)
        {
            switch (parsingState)
            {
                case WATI_BYTES0:
                    check_num = 0x00;
                    boost::asio::read(*serialPtr.get(), boost::asio::buffer(&_buffer[0], 1), errorCode);
                    if(errorCode)
                    {
                        std::cout << boost::system::system_error(errorCode).what() << std::endl;
                        break;
                    }
                    parsingState = _buffer[0] == HEAD_BYTE1 ? WATI_BYTES1 : WATI_BYTES0;
                    if(parsingState == WATI_BYTES0)
                        ROS_DEBUG_STREAM("recv bytes0 error : ->"<<(int)_buffer[0]);
                    break;

                case WATI_BYTES1:
                    boost::asio::read(*serialPtr.get(),boost::asio::buffer(&_buffer[1],1),errorCode);
                    parsingState = _buffer[1] == HEAD_BYTE2 ? WATI_BYTES2_SIZE : WATI_BYTES0;
                    if(parsingState == WATI_BYTES0)
                        ROS_DEBUG_STREAM("recv bytes1  error : ->"<<(int)_buffer[1]);
                    break;

                case WATI_BYTES2_SIZE:
                    boost::asio::read(*serialPtr.get(),boost::asio::buffer(&_buffer[2],1),errorCode);
                    _size = _buffer[2] - 4;
                    parsingState = WATI_BYTES3_MSG_AND_TYPE;
                    break;

                case WATI_BYTES3_MSG_AND_TYPE:
                    boost::asio::read(*serialPtr.get(),boost::asio::buffer(&_buffer[3],_size),errorCode);
                    port_msgType = _buffer[3];
                    parsingState = WATI_CHECK_SUM;
                    break;

                case WATI_CHECK_SUM:
                    boost::asio::read(*serialPtr.get(),boost::asio::buffer(&_buffer[3+_size],1),errorCode);
                    HandlePort::checkSum(_buffer,3+_size,check_num);
                    parsingState = _buffer[3+_size] == check_num ? WATI_PARING_DATA : WATI_BYTES0;
                    if(parsingState == WATI_BYTES0)
                        ROS_DEBUG_STREAM("check sum error! recv is  : ->"<<(int)_buffer[3+_size]<<"  calc is "<<check_num);
                    break;

                case WATI_PARING_DATA:
                    parsingData(port_msgType, _buffer,port_id);
                    parsingState = WATI_BYTES0;
                    break;
            
                default:
                    parsingState = WATI_BYTES0;
                    break;
            }
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
    }
    catch (boost::system::system_error &e)
    {
        std::cout << "Exception Error:" << e.what() << std::endl;
    }
}


void SerialInterface::parsingData(uint8_t port_msgType, uint8_t* _buffer,int port_id)
{
    switch (port_msgType)
    {
        case MSG_BATTERY:
            //ROS_INFO("receive MSG_BATTERY !");
            m_handlePortptr->handleBatteryData(_buffer);
            m_battertPubPtr->Publish(m_handlePortptr->m_batteryMsg.data);
           break;
        case MSG_IMU:
            // ROS_INFO("receive MSG_IMU !");
             m_handlePortptr->handleImuData(_buffer,m_imuCalibData);
             m_imuPubPtr->doPublish(m_handlePortptr->m_imuMsg,m_handlePortptr->m_magMsg,ros::Time::now());
            break;
        case MSG_ULTROSOUND:
            // ROS_INFO("receive MSG_ULTROSOUND !");
             m_handlePortptr->handleSonarData(_buffer,port_id);
             m_sonarPubPtr->doPublish(m_handlePortptr->m_sonarVector,port_id);
             break;

        case MSG_WHEEL_COUNT:
            if(port_id==1)
            {   
               // ROS_INFO("receive LEFT MSG_WHEEL_COUNT !");
                getCopy(_buffer,m_bufLeftGet,14);
                LEFT_GET=1;
                boost::mutex::scoped_lock scoped_lock(m_mutexSend);
                while(LEFT_GET==0 || RIGHT_GET==0)
                    m_condvar.wait(scoped_lock);

                {
                    LEFT_GET=0;
                    RIGHT_GET=0;
                    m_handlePortptr->mergeWheelData(m_bufLeftGet,m_bufRightGet,m_encoderData);
                    m_lrTicksPubPtr->Publish(m_encoderData);
                    m_handlePortptr->handleWheelData(m_encoderData);
                    if(is_publish_odom_transform)
                        m_tfBroadcasePtr->sendTransform(m_handlePortptr->m_odomTransform,ros::Time::now());
                    m_odomPubPtr->doPublish(m_handlePortptr->m_odomMsg,ros::Time::now());
                }
            }
            if(port_id==2)
            {
               // ROS_INFO("receive RIGHT MSG_WHEEL_COUNT !");
                getCopy(_buffer,m_bufRightGet,14);
                boost::mutex::scoped_lock scoped_lock(m_mutexSend);
                {  RIGHT_GET=1;  }
                m_condvar.notify_one();
            }
            break;

        case MSG_LOOP_TEST:
            printf("Left is %d  ",(short)(_buffer[4]*256+_buffer[5]));
            printf("Right is %d  \n",(short)(_buffer[6]*256+_buffer[7]));
            break;
        default:
            break;
    }
}

}
