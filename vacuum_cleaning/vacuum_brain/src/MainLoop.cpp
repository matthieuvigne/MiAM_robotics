/// \file MainLoop.c
/// \brief Main function: code for inverted pendulum.
///
/// \author Matthieu Vigne
/// \copyright GNU GPLv3

#include "LoggerFields.h"
#include "ArduinoListener.h"
#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/trajectory/ThreeWheelsKinematics.hpp>
//~ #include "Utilities.h"

#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <cmath>


#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>



int read_timeout(int const& file, void *buffer, size_t const& size, uint const& timeoutMs)
{
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000 * timeoutMs;
    fd_set set;
    FD_ZERO(&set);
    FD_SET(file, &set);

    int nFiles = select(file + 1, &set, NULL, NULL, &timeout);
    // If there is something to read, read and return the number of bytes read.
    if (nFiles > 0)
        return read(file, buffer, size);
    else
        // Nothing to read: return the return value of select: 0 if timeout, -1 on error.
        return nFiles;
}


/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;
    bytes = read_timeout(fd, event, sizeof(*event), 1);

    if (bytes < 0)
        return -1;
    else if (bytes == sizeof(*event))
        return 1;
    return 0;
}

/**
 * Returns the number of axes on the controller or 0 if an error occurs.
 */
size_t get_axis_count(int fd)
{
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

/**
 * Returns the number of buttons on the controller or 0 if an error occurs.
 */
size_t get_button_count(int fd)
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

/**
 * Current state of an axis.
 */
struct axis_state {
    double x, y, z;
};

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 3;

    if (axis < 2)
    {
        if (event->number % 3 == 0)
            axes[axis].x = event->value / 32768.0;
        else if (event->number % 3 == 1)
            axes[axis].y = event->value / 32768.0;
        else
            axes[axis].z = event->value / 32768.0;
    }

    return axis;
}



// Update loop frequency, in s.
//~ double const LOOP_PERIOD = 0.005;
double const LOOP_PERIOD = 0.010;

int const ENCODER_RESOLUTION = 1600; // Encoder resolution

int main(int argc, char **argv)
{
    // Init raspberry serial ports and GPIO.
    //~ RPi_enablePorts();

    // Create kinematics object representing the robot.
    omni::ThreeWheelsKinematics kinematics(0.165, 0.05);

    // Init log
    Logger logger = initLog();

    // Init communication with arduino
    ArduinoListener arduino(kinematics, ENCODER_RESOLUTION);
    bool isArduinoInit = arduino.initialize("/dev/arduinoUno");

    if (!isArduinoInit)
    {
        std::cout << "Failed to init communication with Arduino" << std::endl;
        exit(0);
    }


    struct js_event event;
    struct axis_state axes[2] = {0};
    size_t axis;

    int fileDescriptor = open("/dev/input/js0", O_RDONLY);

    if (fileDescriptor < 0)
    {
        std::cout << "Error: could not connect to joystick." << std::endl;
        exit(0);
    }
    std::cout << "started" << std::endl;

    // Configure and start main loop.
    Metronome metronome(LOOP_PERIOD * 1e9);
    double currentTime = 0;
    double lastTime = 0;
    omni::BaseSpeed targetSpeed, currentSpeed;
    omni::WheelSpeed targetWheelSpeed, currentWheelSpeed;

    double const MAX_VELOCITY = 0.5;
    // double const MAX_VELOCITY = 0.3;
    double const MAX_ANGULAR_VELOCITY = 4.0;
    double const DEADZONE = 0.15;

    while (true)
    {
        // Wait for next tick.
        lastTime = currentTime;
        metronome.wait();
        currentTime = metronome.getElapsedTime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        int readStatus = read_event(fileDescriptor, &event);
        if (readStatus < 0)
        {
            std::cout << "Lost connection to joystick." << std::endl;
            exit(0);
        }
        else if (readStatus == 1)
        {
            // Parse event
            switch (event.type)
            {
                case JS_EVENT_BUTTON:
                    std::cout << "Button " << int(event.number) <<  (event.value ? " pressed" : " released");
                    break;
                case JS_EVENT_AXIS:
                    get_axis_state(&event, axes);
                    std::cout << "Axis " << int(event.number) <<  (event.value ? " pressed" : " released");

                    // Compute target velocity.
                    // Robot: x is forward, y is left
                    targetSpeed.vx_ = -axes[0].y;
                    targetSpeed.vy_ = -axes[0].x;
                    targetSpeed.omega_ = -axes[1].x;

                    if (std::fabs(targetSpeed.vx_) < DEADZONE)
                        targetSpeed.vx_ = 0;
                    if (std::fabs(targetSpeed.vy_) < DEADZONE)
                        targetSpeed.vy_ = 0;
                    if (std::fabs(targetSpeed.omega_) < DEADZONE)
                        targetSpeed.omega_ = 0;
                    targetSpeed.vx_ *= MAX_VELOCITY;
                    targetSpeed.vy_ *= MAX_VELOCITY;
                    targetSpeed.omega_ *= MAX_ANGULAR_VELOCITY;

                default:
                    /* Ignore init events. */
                    break;
            }
        }

        //~ std::cout << "Target speed " << targetSpeed.vx_ << " " << targetSpeed.vy_ << " " << targetSpeed.omega_ << std::endl;;

        arduino.setTarget(targetSpeed);
        currentSpeed = arduino.getCurrentSpeed();

        // Log
        logger.setData(LOGGER_TIME, currentTime);
        logger.setData(LOGGER_TARGET_VELOCITY_X, targetSpeed.vx_);
        logger.setData(LOGGER_TARGET_VELOCITY_Y, targetSpeed.vy_);
        logger.setData(LOGGER_TARGET_VELOCITY_OMEGA, targetSpeed.omega_);
        logger.setData(LOGGER_CURRENT_VELOCITY_X, currentSpeed.vx_);
        logger.setData(LOGGER_CURRENT_VELOCITY_Y, currentSpeed.vy_);
        logger.setData(LOGGER_CURRENT_VELOCITY_OMEGA, currentSpeed.omega_);

        targetWheelSpeed = kinematics.inverseKinematics(targetSpeed);
        currentWheelSpeed = kinematics.inverseKinematics(currentSpeed);

        logger.setData(LOGGER_TARGET_WHEEL_VELOCITY_1, targetWheelSpeed.w_[0]);
        logger.setData(LOGGER_TARGET_WHEEL_VELOCITY_2, targetWheelSpeed.w_[1]);
        logger.setData(LOGGER_TARGET_WHEEL_VELOCITY_3, targetWheelSpeed.w_[2]);
        logger.setData(LOGGER_CURRENT_WHEEL_VELOCITY_1, currentWheelSpeed.w_[0]);
        logger.setData(LOGGER_CURRENT_WHEEL_VELOCITY_2, currentWheelSpeed.w_[1]);
        logger.setData(LOGGER_CURRENT_WHEEL_VELOCITY_3, currentWheelSpeed.w_[2]);
        logger.writeLine();
    }
    return 0;
}

