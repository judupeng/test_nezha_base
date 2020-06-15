# ifndef __EEZHA_BASE_MSG_MSG_DEFINE_H__
# define __EEZHA_BASE_MSG_MSG_DEFINE_H__

#include <ros/ros.h>


#define MSG_BATTERY     0x12
#define MSG_WHEEL_COUNT 0x10
#define MSG_IMU         0x03
#define MSG_INFRARED    0x05
#define MSG_ULTROSOUND  0x60

#define SERVMSG 0x70

#define MSG_receve_test 0x01
#define MSG_LOOP_TEST   0x31


#define HEAD_BYTE1 0xAA
#define HEAD_BYTE2 0x55

#define SEND_SPEED 0x50
#define SEND_PID   0x51
#define SEND_STEER 0x52

enum ParsingState
{
    WATI_BYTES0,
    WATI_BYTES1,
    WATI_BYTES2_SIZE,
    WATI_BYTES3_MSG_AND_TYPE,
    WATI_CHECK_SUM,
    WATI_PARING_DATA
};


enum CarMode
{
    Differential_Drive,
    Three_wheel_omnidirectional,
    Four_wheel_omnidirectional ,
    Four_wheeled_Mecanum ,
    Four_wheel_sliding ,
    Four_wheeled_Ackerman
};


struct Encoder
{
    ros::Time last_twist_time_;
    ros::Time last_time_;
    ros::Time now_;
    ros::Time check_time_;
    
    double wheel_distance_;//左右轮间距
    double pid_rate_;
    double wheel_track_;// 前后轮间距
    double delta_time_;
    double delta_left_; //左轮走过的距离
    double delta_right_; //右轮走过的距离
    double delta_xy_ave_;
    double delta_th_;
    double delta_x_;
    double delta_y_;
    double v_linear_;
    double v_angular_;
    double accumulation_x_;
    double accumulation_y_;
    double accumulation_th_;
    double accel_limit_;
    double max_accel_;
    double max_linear_speed_;
    double max_angular_speed_;
    double linear_correction_factor_;
    double max_steering_angle;
    double steering_offset_factor;
    double current_steering_angle_;
    double set_servo_steering_angle;
    double servo_bias_;
     //*解决定时器溢出 start
    double encoder_resolution_;
    double ticks_per_meter_;
    double wheel_diameter_;
    double gear_reduction_;
    double encoder_low_wrap_;
    double encoder_high_wrap_;
    double encode_sampling_time_;
    // end
    int cur_left_;
    int cur_right_;
    int32_t rev_left_;
    int32_t rev_right_;
    int kp_;
    int ki_;
    int kd_;
//哪吒的两条腿分开 start
    int32_t total_angle_left_;  //单位是tick 该电机共8192个tick
    int32_t total_angle_right_;
    int16_t speed_rpm_left_;
    int16_t speed_rpm_right_;
// end

// start
    CarMode carMode;
//end
    int    encoder_min_;
    int    encoder_max_;
    int    l_wheel_mult_;
    int    r_wheel_mult_; 
    bool start_flag_;
};




#endif
