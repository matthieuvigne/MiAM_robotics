/// A GUI to test  and configure the STS3215 servos of the robot

#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/STSServoDriver.h>
#include "DH_transform.hpp"

#include <cstdlib>
#include <iostream>
#include <unistd.h>

using namespace kinematics;

double radToAngle(double const& rad)
{
    return 2048 + (2048 * rad / M_PI);
}

double modulo(double const& rad)
{
    double x = rad;
    while (x < -M_PI)
        x += 2 * M_PI;
    while (x > M_PI)
        x -= 2 * M_PI;
    return x;
}

int main (int argc, char *argv[])
{
    // Try to communicate with servo.

    // RPi_enablePorts();
    STSServoDriver driver;

    if (!driver.init("/dev/ttyUSB0", 17))
    {
        std::cout << "Failed to init communication with servos." << std::endl;
        return 0;
    }
    // Start all servos in position mode.
    driver.setMode(0xFE, STS::Mode::POSITION);
    usleep(10000);

    DHTransformVector armKinematics = create_main_robot_arm();

    for (int i = 0; i < 4; i++)
    {
        double angle = armKinematics.at(i).get_parameter(DHTransform::Parameter::a2);
        std::cout << "Target parameter:" << std::to_string(i) << "rad: " << angle << " count: " << radToAngle(angle);
        std::cout << " current position:" << driver.getCurrentPosition(10 + i) << std::endl;
        driver.setTargetPosition(10 + i, radToAngle(-angle));
    }

    usleep(2000000);
    Eigen::Affine3d trans =
      Eigen::Translation3d(Eigen::Vector3d(0.200, 0.000, -0.180))
    * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix();

    // Bras au sol devant le robot
    trans.translation().x() = 0.200;
    trans.translation().y() = 0.000;
    trans.translation().z() = -0.180;


    std::cout << trans.matrix() << std::endl;

    std::vector<double *> args({trans.matrix().data()});
    armKinematics.optimize_parameters(kinematics::DHTransformVector::ProblemType::FullPose, args.data(), true);



    std::cout << trans.matrix() << std::endl;

    std::cout << armKinematics.get_global_transform().matrix() << std::endl;

    for (int i = 0; i < 4; i++)
    {
        double angle = modulo(armKinematics.at(i).get_parameter(DHTransform::Parameter::a2));
        std::cout << "Target parameter:" << std::to_string(i) << "rad: " << angle << " count: " << radToAngle(angle);
        std::cout << " current position:" << driver.getCurrentPosition(10 + i) << std::endl;
        driver.setTargetPosition(10 + i, radToAngle(-angle));
    }
    usleep(5000000);


    trans.translation().x() = 0.200;
    trans.translation().y() = 0.000;
    trans.translation().z() = -0.14;

    std::vector<double *> args2{trans.matrix().data()};

    std::cout << armKinematics.print() << std::endl;
    int num_iters = armKinematics.optimize_parameters(kinematics::DHTransformVector::ProblemType::FullPose, args2.data(), true);
    std::cout << "Num iters: " << num_iters << std::endl;
    return EXIT_SUCCESS;

    for (int i = 0; i < 4; i++)
    {
        double angle = modulo(armKinematics.at(i).get_parameter(DHTransform::Parameter::a2));
        std::cout << "Target parameter:" << std::to_string(i) << "rad: " << angle << " count: " << radToAngle(angle);
        std::cout << " current position:" << driver.getCurrentPosition(10 + i) << std::endl;
        // driver.setTargetPosition(10 + i, radToAngle(-angle));
    }

    usleep(10000000);
    // Stop all motors
    driver.disable(0xFE);
    return 0;
}


