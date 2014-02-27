#include <gtest/gtest.h>
#include <snark/actuators/wheels/wheel_command.h>

using namespace snark::wheels;

TEST(wheels, wheel_command_test1)
{
    Eigen::Matrix4d wheel_pose = ( Eigen::Matrix4d() << -0.707107, 0, -0.707107, 0.687035, 0.707107, 0, -0.707107, 0.687035, 0, -1, 0, -0.15034, 0, 0, 0, 1 ).finished();

    steer_command desired;
    desired.velocity.x() = 1;
    desired.velocity.y() = 0;
    desired.turnrate = 0.174532925;
    wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );

    EXPECT_LT( fabs( wheel_command_.velocity + 0.905587 ), 1e-3 );
    EXPECT_LT( fabs( wheel_command_.turnrate + 0.920811279 ), 1e-3 );
}

TEST(wheels, wheel_command_test2)
{
    Eigen::Matrix4d wheel_pose = ( Eigen::Matrix4d() << 1, 0, 0, -1, 0, 0, 1, -1, 0, -1, 0, 0, 0, 0, 0, 1 ).finished();

    steer_command desired;
    desired.velocity.x() = -1;
    desired.velocity.y() = 1;
    desired.turnrate = 0;
    wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
    std::cout<<"v: "<<wheel_command_.velocity<<std::endl<<"angle: "<<wheel_command_.turnrate<<std::endl;
}

TEST(wheels, wheel_command_test3)
{
    Eigen::Matrix4d wheel_pose = ( Eigen::Matrix4d() << -1, 0, 0, -1, 0, 0, -1, 1, 0, -1, 0, 0, 0, 0, 0, 1 ).finished();

    steer_command desired;
    desired.velocity.x() = 0;
    desired.velocity.y() = 0;
    desired.turnrate = 0.1;
    wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
    std::cout<<"v: "<<wheel_command_.velocity<<std::endl<<"angle: "<<wheel_command_.turnrate<<std::endl;
}
