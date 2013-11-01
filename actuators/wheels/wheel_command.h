#ifndef WHEEL_COMMANDS_H
#define WHEEL_COMMANDS_H
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/math/constants/constants.hpp>
#include <comma/base/exception.h>
#include <snark/math/frame_transforms.h>
#include <snark/math/applications/frame.h>
#include <boost/lexical_cast.hpp>

namespace snark{ namespace wheels{

namespace boost_constants = boost::math::constants;

inline double deg2rad(double theta)
{
    return(boost_constants::pi<double>()*theta/180);
}

inline double rad2deg(double theta)
{
    return(180*theta/boost_constants::pi<double>());
}

const double yaw_rate_tol = 0.1; // rad/s
const double velocity_max = 10; //m/s
const double yaw_rate_max = 90; //deg/s

struct steer_command
{
    Eigen::Vector2d velocity;
    double yaw;
};

struct wheel_command
{
    double velocity;
    double yaw;
};

wheel_command compute_wheel_command( const steer_command& desired , Eigen::Matrix4d wheel_pose_, double wheel_offset = 0 );

}}//namespace snark{ namespace wheels{


#endif
