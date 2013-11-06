#include "wheel_command.h"

namespace snark{ namespace wheels{

using namespace frame_transforms;

snark::wheels::wheel_command compute_wheel_command(const steer_command &desired , Eigen::Matrix4d wheel_pose_, double wheel_offset)
{
    wheel_command command_;
    if(desired.velocity.norm()>velocity_max)
    {
        COMMA_THROW( comma::exception,  "velocity must be between: " << -velocity_max << " and " << velocity_max << ", input value was: " << desired.velocity.norm());
    }

    if(fabs(desired.yaw)>yaw_rate_max)
    {
        COMMA_THROW( comma::exception, "yaw rate must be between " << -yaw_rate_max << " and " << yaw_rate_max << ", input value was: " << desired.yaw );
    }

    //convert to radians first
    double yaw_rate=deg2rad(desired.yaw);

    // a very small non zero yaw rate will cause calculation errors
    if(fabs(yaw_rate)<yaw_rate_tol)
    {
        command_.velocity=desired.velocity.norm();
        command_.yaw=atan2(desired.velocity.y(),desired.velocity.x());
        //limit movement to -pi/2 and pi/2
        if(command_.yaw>boost_constants::pi<double>()/2)
        {
            command_.yaw-=boost_constants::pi<double>();
            command_.velocity=-command_.velocity;
        }
        else if(command_.yaw<-boost_constants::pi<double>()/2)
        {
            command_.yaw+=boost_constants::pi<double>();
            command_.velocity=-command_.velocity;
        }
    }
    else
    {
        // find ICR position, ICR is at a 90 deg rotation from velocity vector
        Eigen::Vector4d ICRPosition;
        ICRPosition<<-desired.velocity.y()/yaw_rate,desired.velocity.x()/yaw_rate,0,1;

        //ICR in wheel steering axis frame
        ICRPosition=inverse_transform(wheel_pose_)*ICRPosition;

        //transform ICR to wheel coordinates
        command_.yaw=atan2(ICRPosition(0),ICRPosition(2)); // take x and z positions only
        command_.velocity=yaw_rate*((ICRPosition).norm()-wheel_offset);

        //limit movement to -pi/2 and pi/2
        if(command_.yaw>boost_constants::pi<double>()/2)
        {
            command_.yaw-=boost_constants::pi<double>();
            command_.velocity=-yaw_rate*((ICRPosition).norm()+wheel_offset);
        }
        else if(command_.yaw<-boost_constants::pi<double>()/2)
        {
            command_.yaw+=boost_constants::pi<double>();
            command_.velocity=-yaw_rate*((ICRPosition).norm()+wheel_offset);
        }
    }

    //convert command to degrees
    command_.yaw=rad2deg(command_.yaw);
    return(command_);
}

}}
