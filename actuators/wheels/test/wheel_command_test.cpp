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

TEST( wheels, wheel_command_turn )
{
    Eigen::Matrix4d wheel_pose = ( Eigen::Matrix4d() << 1, 0, 0, -2, 0, 0, 1, -3, 0, -1, 0, 0, 0, 0, 0, 1 ).finished();

    steer_command desired( 5, 0, 0.6 );

    {
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose );
        EXPECT_NEAR( 6.90507, wheel_command_.velocity, 1e-6 );
        EXPECT_NEAR( -0.1746722, wheel_command_.turnrate, 1e-6 );
    }

    {
        wheel_command wheel_command_ = compute_wheel_command( desired, wheel_pose, 0.15 );
        EXPECT_NEAR( 6.81507, wheel_command_.velocity, 1e-6 );
        EXPECT_NEAR( -0.1746722, wheel_command_.turnrate, 1e-6 );
    }
}

