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

TEST(wheels, wheel_command_rear_left)
{
    Eigen::Matrix4d wheel_pose = ( Eigen::Matrix4d() << -1, 0, 0, -1, 0, 0, -1, 1, 0, -1, 0, 0, 0, 0, 0, 1 ).finished();

    std::cout << "rear left: " << std::endl << wheel_pose << std::endl;

    {
        steer_command desired( -1, 1, 0 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "move SW:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }

    {
        steer_command desired( 0, 1, 0 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "move W:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }

    {
        steer_command desired( 0, 0, -0.1 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "turn CW:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }

    {
        steer_command desired( 0, 0, 0.1 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "turn CCW:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }
}

TEST(wheels, wheel_command_front_left)
{
    Eigen::Matrix4d wheel_pose = ( Eigen::Matrix4d() << -1, 0, 0, 1, 0, 0, -1, 1, 0, -1, 0, 0, 0, 0, 0, 1 ).finished();

    std::cout << "front left: " << std::endl << wheel_pose << std::endl;

    {
        steer_command desired( -1, 1, 0 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "move SW:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }

    {
        steer_command desired( 0, 1, 0 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "move W:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }

    {
        steer_command desired( 0, 0, -0.1 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "turn CW:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }

    {
        steer_command desired( 0, 0, 0.1 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "turn CCW:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }
}

TEST(wheels, wheel_command_front_right)
{
    Eigen::Matrix4d wheel_pose = ( Eigen::Matrix4d() << 1, 0, 0, 1, 0, 0, 1, -1, 0, -1, 0, 0, 0, 0, 0, 1 ).finished();

    std::cout << "front right: " << std::endl << wheel_pose << std::endl;

    {
        steer_command desired( -1, 1, 0 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "move SW:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }

    {
        steer_command desired( 0, 1, 0 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "move W:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }

    {
        steer_command desired( 0, 0, -0.1 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "turn CW:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }

    {
        steer_command desired( 0, 0, 0.1 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "turn CCW:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }
}

TEST(wheels, wheel_command_rear_right)
{
    Eigen::Matrix4d wheel_pose = ( Eigen::Matrix4d() << 1, 0, 0, -1, 0, 0, 1, -1, 0, -1, 0, 0, 0, 0, 0, 1 ).finished();

    std::cout << "rear right: " << std::endl << wheel_pose << std::endl;

    {
        steer_command desired( -1, 1, 0 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "move SW:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }

    {
        steer_command desired( 0, 1, 0 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "move W:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }

    {
        steer_command desired( 0, 0, -0.1 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "turn CW:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }

    {
        steer_command desired( 0, 0, 0.1 );
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        std::cout << "turn CCW:  velocity: " << wheel_command_.velocity << ", angle: " << wheel_command_.turnrate << std::endl;
    }
}

