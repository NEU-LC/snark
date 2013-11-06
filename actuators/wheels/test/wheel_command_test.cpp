#include <iostream>
#include <gtest/gtest.h>
#include <snark/actuators/wheels/wheel_command.h>

using namespace snark::wheels;

TEST(wheels, wheel_command_test)
{
    Eigen::Matrix4d wheel_pose = (Eigen::Matrix4d()<<-0.707107,0,-0.707107,0.687035,0.707107,0,-0.707107,0.687035,0,-1,0,-0.15034,0,0,0,1).finished();

    steer_command desired;
    desired.velocity.x()=1;
    desired.velocity.y()=0;
    desired.yaw=10;
    wheel_command wheel_command_=compute_wheel_command(desired,wheel_pose);
    std::cout<< "Front left wheel command for velocity: "<<desired.velocity.x()<<", "<<desired.velocity.y()<<" and turn rate "<<desired.yaw<<" is: "<<wheel_command_.velocity<<", "<<wheel_command_.yaw<<std::endl;
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
