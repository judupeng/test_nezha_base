#include "nezha_base/serial/handle_port.h"
#include "nezha_base/sender/debug_publisher.h"
#include <tf/transform_datatypes.h>

namespace base {


HandlePort::HandlePort()
{
    imu_msg_count_=0;
    ax_roll_=0;
    ax_pitch_=0;
    ax_yaw_=0;
    m_sonarMsg.radiation_type=m_sonarMsg.ULTRASOUND;
    m_sonarMsg.field_of_view=M_PI/6;
};
//  用于哪吒两条腿
void HandlePort::mergeWheelData(uint8_t* buffer_left,uint8_t* buffer_right,Encoder & encoder_data)
{
    //rev_left_就是total angle left，单位是tick  该电机共8192个tick
    encoder_data.rev_left_ = (int32_t)((buffer_left[4]<< 24)|(buffer_left[5] << 16)|(buffer_left[6] << 8)| buffer_left[7]);
    encoder_data.rev_right_=-1*(int32_t)((buffer_right[4]<< 24)|(buffer_right[5]<< 16)|(buffer_right[6]<< 8)|buffer_right[7]);
    encoder_data.speed_rpm_left_ = (int16_t)((buffer_left[8]<< 8)| buffer_left[9]);
    encoder_data.speed_rpm_right_= (int16_t)((buffer_right[8]<< 8)| buffer_right[9]);
    //ROS_INFO(" rpm left= %d ; rpm right = %d",encoder_data.speed_rpm_left_,encoder_data.speed_rpm_right_);
    ROS_INFO(" total left = %d ;  total right = %d \n",encoder_data.rev_left_,encoder_data.rev_right_);
}

//  用于哪吒两条腿

void HandlePort::handleWheelData(Encoder & encoder_data)
{
   
    encoder_data.now_ = ros::Time::now();
    double delta_time_ = (encoder_data.now_ - encoder_data.last_time_).toSec();
    double v_linear_=0;
    double v_angular_=0;
    double delta_x_=0;
    double delta_y_=0;
    if(encoder_data.start_flag_)
    {
        encoder_data.accumulation_y_ = 0.0;
        encoder_data.accumulation_x_ = 0.0;
        encoder_data.accumulation_th_ = 0.0;

        encoder_data.cur_left_ = encoder_data.rev_left_;
        encoder_data.cur_right_ = encoder_data.rev_right_;

        encoder_data.last_time_ = encoder_data.now_;
        encoder_data.start_flag_ = false;
        ROS_INFO("Robot is Successfuly connected!");
        
    }

    if(encoder_data.cur_left_ == 0 && encoder_data.cur_right_ == 0)
    {
        encoder_data.delta_left_ = 0;
        encoder_data.delta_right_ = 0;
    }
    else
    {
        if(encoder_data.rev_left_ < encoder_data.encoder_low_wrap_ && encoder_data.cur_left_ > encoder_data.encoder_high_wrap_)
            encoder_data.l_wheel_mult_++;
        else if(encoder_data.rev_left_ > encoder_data.encoder_high_wrap_ && encoder_data.cur_left_ < encoder_data.encoder_low_wrap_)
            encoder_data.l_wheel_mult_--;
        else
            encoder_data.l_wheel_mult_ = 0;
        if(encoder_data.rev_right_ < encoder_data.encoder_low_wrap_ && encoder_data.cur_right_ > encoder_data.encoder_high_wrap_)
            encoder_data.r_wheel_mult_++;
        else if(encoder_data.rev_right_ > encoder_data.encoder_high_wrap_ && encoder_data.cur_right_ < encoder_data.encoder_low_wrap_)
            encoder_data.r_wheel_mult_--;
        else
            encoder_data.r_wheel_mult_ = 0;
        // delta_left_ 单位米，左轮走过的距离

        encoder_data.delta_left_  = 1.0*(encoder_data.rev_left_ + encoder_data.l_wheel_mult_ * (encoder_data.encoder_max_ - encoder_data.encoder_min_)-encoder_data.cur_left_)/encoder_data.ticks_per_meter_;
        encoder_data.delta_right_ = 1.0*(encoder_data.rev_right_ +encoder_data.r_wheel_mult_ * (encoder_data.encoder_max_ - encoder_data.encoder_min_)-encoder_data.cur_right_)/encoder_data.ticks_per_meter_;
        ROS_INFO("delta_left_ = %4.4f, delta_right_= %4.4f \n",encoder_data.delta_left_,encoder_data.delta_right_);
 }
    //  rev_left_ 收到的左编码器刻度位置（ticks位置）
    encoder_data.cur_left_ = encoder_data.rev_left_;
    encoder_data.cur_right_ = encoder_data.rev_right_;

    encoder_data.delta_xy_ave_ = (encoder_data.delta_left_ + encoder_data.delta_right_)/2.0;
    switch(encoder_data.carMode)
    {
        case Differential_Drive:
        {
            //  两轮差速 start
            encoder_data.delta_th_ = (encoder_data.delta_right_ - encoder_data.delta_left_)/encoder_data.wheel_distance_;
            v_linear_ = encoder_data.delta_xy_ave_ / delta_time_;
            v_angular_= encoder_data.delta_th_ / delta_time_;
            //  两轮差速 end
            break;
        }
        case Four_wheeled_Ackerman:
        {
            //  阿克曼轮 start
            v_linear_ = encoder_data.delta_xy_ave_ / delta_time_;
            //ROS_INFO("v_linear_ = %4.4f",v_linear_);
            v_angular_ = (v_linear_*tan(encoder_data.current_steering_angle_))/encoder_data.wheel_track_;
            encoder_data.delta_th_ = v_angular_*delta_time_;
            // 阿克曼轮 end
           break;
        }
    }
   
    if(encoder_data.delta_xy_ave_ != 0)
    {
        delta_x_ = cos(encoder_data.accumulation_th_ + encoder_data.delta_th_) * encoder_data.delta_xy_ave_;
        delta_y_ = sin(encoder_data.accumulation_th_ + encoder_data.delta_th_) * encoder_data.delta_xy_ave_;
        encoder_data.accumulation_x_ += delta_x_;
        encoder_data.accumulation_y_ += delta_y_;
    }
    encoder_data.accumulation_th_ += encoder_data.delta_th_;
    m_odomTransform.transform.translation.x = encoder_data.accumulation_x_;
    m_odomTransform.transform.translation.y = encoder_data.accumulation_y_;
    m_odomTransform.transform.translation.z = 0.0;
    tf::Quaternion q;
    q.setRPY(0,0,encoder_data.accumulation_th_);
    m_odomTransform.transform.rotation.x = q.x();
    m_odomTransform.transform.rotation.y = q.y();
    m_odomTransform.transform.rotation.z = q.z();
    m_odomTransform.transform.rotation.w = q.w();
/////////////////////////////
    m_odomMsg.header.stamp    = encoder_data.now_;
    m_odomMsg.pose.pose.position.x = encoder_data.accumulation_x_;
    m_odomMsg.pose.pose.position.y = encoder_data.accumulation_y_;
    m_odomMsg.pose.pose.position.z = 0;
    m_odomMsg.pose.pose.orientation.x = q.getX();
    m_odomMsg.pose.pose.orientation.y = q.getY();
    m_odomMsg.pose.pose.orientation.z = q.getZ();
    m_odomMsg.pose.pose.orientation.w = q.getW();
    m_odomMsg.twist.twist.linear.x = v_linear_;
    m_odomMsg.twist.twist.linear.y = 0;
    m_odomMsg.twist.twist.angular.z = v_angular_;
    m_odomMsg.twist.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                                   0, 1e-3, 1e-9, 0, 0, 0, 
                                   0, 0, 1e6, 0, 0, 0,
                                   0, 0, 0, 1e6, 0, 0, 
                                   0, 0, 0, 0, 1e6, 0, 
                                   0, 0, 0, 0, 0, 1e-9 };
    m_odomMsg.pose.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                                   0, 1e-3, 1e-9, 0, 0, 0, 
                                   0, 0, 1e6, 0, 0, 0,
                                   0, 0, 0, 1e6, 0, 0, 
                                   0, 0, 0, 0, 1e6, 0, 
                                   0, 0, 0, 0, 0, 1e-9 };
    encoder_data.last_time_ =encoder_data.now_;
};


void HandlePort::handleImuData(uint8_t* buffer_data,sensor_msgs::Imu imuCalibData )
{
    ImuData imu_data_; 
    double ax_cm_k_=0.08;
    

                                                                                        //2000⋅3.1415926/180/32768=0.0010652644
    imu_data_.angular_velocity.x  = (float)((int16_t)(buffer_data[4]*256+buffer_data[5]))*0.0010652644; // 乘以2000为陀螺仪的量程-2000～1999度/秒，结果单位为弧度/秒。
    imu_data_.angular_velocity.y  = (float)((int16_t)(buffer_data[6]*256+buffer_data[7]))*0.0010652644; // 除以32768，-32768～32767为int16_t类型，占用16位的最大范围
    imu_data_.angular_velocity.z  = (float)((int16_t)(buffer_data[8]*256+buffer_data[9]))*0.0010652644;
    imu_data_.linear_acceleration.x = (float)((int16_t)(buffer_data[10]*256+buffer_data[11]))*0.00059814453; //设置的量程是2G，结果单位为1，大小范围：（-2～+2）*9.8
    imu_data_.linear_acceleration.y = (float)((int16_t)(buffer_data[12]*256+buffer_data[13]))*0.00059814453;// 2⋅9.8/32768=  0.00059814453
    imu_data_.linear_acceleration.z = (float)((int16_t)(buffer_data[14]*256+buffer_data[15]))*0.00059814453;
    if(imu_msg_count_>200)
    { // 互补滤波  angle= angle_Acc*
        double ax_roll_acc_ = -(atan2(imuCalibData.linear_acceleration.x,imuCalibData.linear_acceleration.z));//加速度计（陀螺仪）正前方为Y轴正，右方为x轴正，上方为Z轴正
        ax_roll_ = ax_cm_k_*ax_roll_acc_+(1-ax_cm_k_)*(ax_roll_+(imuCalibData.angular_velocity.y)*0.01);// 0.01s为IMU采样时间，100HZ
        double ax_pitch_acc_ = -(atan2(imuCalibData.linear_acceleration.y,imuCalibData.linear_acceleration.z));
        ax_pitch_ = ax_cm_k_*ax_pitch_acc_+(1-ax_cm_k_)*(ax_pitch_+(imuCalibData.angular_velocity.x)*0.01);
        ax_yaw_ = (ax_yaw_ -(imuCalibData.angular_velocity.z)*0.01);
    }
    else
        imu_msg_count_++;
    
    m_imuMsg.orientation = tf::createQuaternionMsgFromRollPitchYaw(ax_roll_,ax_pitch_,ax_yaw_);
    m_imuMsg.angular_velocity.x = imu_data_.angular_velocity.x;
    m_imuMsg.angular_velocity.y = imu_data_.angular_velocity.y;
    m_imuMsg.angular_velocity.z = imu_data_.angular_velocity.z;
    m_imuMsg.linear_acceleration.x = imu_data_.linear_acceleration.x;
    m_imuMsg.linear_acceleration.y = imu_data_.linear_acceleration.y;
    m_imuMsg.linear_acceleration.z = imu_data_.linear_acceleration.z;
    m_imuMsg.orientation_covariance = {0.05, 0, 0,
                                            0, 0.05, 0,
                                            0, 0, 0.05};
    m_imuMsg.angular_velocity_covariance = {0.01, 0, 0,
                                                 0, 0.01, 0,
                                                 0, 0, 0.01};
    m_imuMsg.linear_acceleration_covariance = {-1, 0, 0,
                                                     0, 0, 0,
                                                     0, 0, 0};                                          // 4800/8192=0.5859375
    m_magMsg.magnetic_field.x=(float)((int16_t)((buffer_data[16]<<8) | buffer_data[17]))*0.5859375; // 原始量程为-4800 ~ +4799 uT（微T）
    m_magMsg.magnetic_field.y=(float)((int16_t)((buffer_data[18]<<8) | buffer_data[19]))*0.5859375;//结果单位为高斯磁场强度单位 uT/LSB
    m_magMsg.magnetic_field.z=(float)((int16_t)((buffer_data[20]<<8) | buffer_data[21]))*0.5859375;//除以8192，为int16_t类型,但是占用14位的最大范围-8192～8192

};


void HandlePort::handleSonarData(uint8_t* buffer_data,int port_id)
{
    m_sonarMsg.header.stamp=ros::Time::now();
    std::vector<sensor_msgs::Range>().swap(m_sonarVector);
    for (int i=0; i<2; i++)
    {
        m_sonarMsg.header.frame_id="ultrasound_"+std::to_string(i+2*port_id);
        m_sonarMsg.range=(float)(int16_t)((buffer_data[2*i+4]<<8) | buffer_data[2*i+5]);
        m_sonarVector.push_back(m_sonarMsg);
    }
};


void HandlePort::handleBatteryData(uint8_t* buffer_data)
{
    m_batteryMsg.data = (float)(((buffer_data[4]<<8)+buffer_data[5]))/100;
};


void HandlePort::checkSum(uint8_t* data, size_t len, uint8_t& dest)
{
    dest = 0x00;
    for(int i=0;i<len;i++)
    {
        dest += *(data + i);
    }
};

}

