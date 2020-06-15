#ifndef __EEZHA_BASE_SENDER_DATA_IMU_DATA_H__
#define __EEZHA_BASE_SENDER_DATA_IMU_DATA_H__

#include <ros/ros.h>
#include <cmath>
#include <deque>

namespace base {


class ImuData
{
    struct LinearAcceleration {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        };
    class Orientation {

        public:
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double w = 0.0;
        public:
             void normlize()
             {
                 double tmp= sqrt(pow(x,2)+pow(y,2)+pow(z,2)+pow(w,2));
                 x=x/tmp;
                 y=y/tmp;
                 z=z/tmp;
                 w=w/tmp;
             }
        };

    public:
        static bool syncData(std::deque<ImuData> unsync_data,std::deque<ImuData> synced_data,double sync_time,double tolerate_time=0.2);
    public:
        double time = 0.0;
        LinearAcceleration linear_acceleration;
        AngularVelocity angular_velocity;
        Orientation orientation; 
    
};

}

#endif