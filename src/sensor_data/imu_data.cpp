#include "nezha_base/sensor_data/imu_data.h"


namespace base {


bool ImuData::syncData(std::deque<ImuData> unsync_data,std::deque<ImuData> synced_data,double sync_time,double tolerate_time)
{
    while(unsync_data.size() >=2)
    {
        if(unsync_data.front().time >sync_time)
            return false;
        if(unsync_data.at(1).time < sync_time)
        {
            unsync_data.pop_front();
            continue;
        }
        if((sync_time-unsync_data.front().time) > tolerate_time)
            return false;
        if((sync_time-unsync_data.at(1).time) < -tolerate_time)
            return false;
    }
    if(unsync_data.size() <2)
        return false;
    
    ImuData tmp_data;
    ImuData  first_data=unsync_data.at(0) ;
    ImuData  second_data=unsync_data.at(1) ;
    double   tmp_value= (sync_time-first_data.time)/(second_data.time- first_data.time);

    tmp_data.linear_acceleration.x= first_data.linear_acceleration.x + (second_data.linear_acceleration.x-first_data.linear_acceleration.x)*tmp_value;
    tmp_data.linear_acceleration.y= first_data.linear_acceleration.y + (second_data.linear_acceleration.y-first_data.linear_acceleration.y)*tmp_value;
    tmp_data.linear_acceleration.z= first_data.linear_acceleration.z + (second_data.linear_acceleration.z-first_data.linear_acceleration.z)*tmp_value;

    tmp_data.angular_velocity.x= first_data.angular_velocity.x + (second_data.angular_velocity.x-first_data.angular_velocity.x)*tmp_value;
    tmp_data.angular_velocity.y= first_data.angular_velocity.y + (second_data.angular_velocity.y-first_data.angular_velocity.y)*tmp_value;
    tmp_data.angular_velocity.z= first_data.angular_velocity.z + (second_data.angular_velocity.z-first_data.angular_velocity.z)*tmp_value;

    tmp_data.orientation.x= first_data.orientation.x + (second_data.orientation.x-first_data.orientation.x)*tmp_value;
    tmp_data.orientation.y= first_data.orientation.y + (second_data.orientation.y-first_data.orientation.y)*tmp_value;
    tmp_data.orientation.z= first_data.orientation.z + (second_data.orientation.z-first_data.orientation.z)*tmp_value;
    tmp_data.orientation.w= first_data.orientation.w + (second_data.orientation.w-first_data.orientation.w)*tmp_value;
    tmp_data.orientation.normlize();
    tmp_data.time=sync_time;
    synced_data.push_back(tmp_data);
};











}