// A utility program to test the robot motion on the table, using the arrow keys.
// To compile:
// arm-linux-gnueabihf-g++ TestMotion.cpp ../MainRobotCode/embedded/src/uCListener.cpp -o TestMotion `pkg-config --cflags --libs x11 miam_utils_arm` -I ../MainRobotCode/common/include/  -I ../MainRobotCode/embedded/include/ -pthread
// Note that X11 lib must be installed...
// On the robot: g++ TestMotion.cpp uCListener.cpp -o TestMotion -pthread -I. -L. -lmiam_utils_arm -lX11


#include <X11/Xlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string>
#include <thread>
#include <iomanip>

#include "Parameters.h"
#include "uCListener.h"
#include <miam_utils/miam_utils.h>


const double LOOP_PERIOD = 0.005;

miam::L6470 stepperMotors; ///< Robot driving motors.

void killCode(int x)
{
    stepperMotors.hardStop();
    stepperMotors.highZ();
}

void odometry(miam::RobotPosition currentPosition)
{

    Metronome metronome(LOOP_PERIOD * 1e9);
    double currentTime = 0.0;
    uCData microcontrollerData = uCListener_getData();


    DrivetrainKinematics kinematics = DrivetrainKinematics(robotdimensions::wheelRadius,
                                       robotdimensions::wheelSpacing,
                                       robotdimensions::encoderWheelRadius,
                                       robotdimensions::encoderWheelSpacing);
    std::cout << std::setprecision(2);
    while (true)
    {
        // Wait for next tick.
        double lastTime = currentTime;
        metronome.wait();
        currentTime = metronome.getElapsedTime();
        double dt = currentTime - lastTime;

        // Odometry

        uCData oldData = microcontrollerData;
        microcontrollerData = uCListener_getData();

        // Compute encoder update.
        WheelSpeed encoderIncrement;
        encoderIncrement.right = microcontrollerData.encoderValues[0] - oldData.encoderValues[0];
        encoderIncrement.left = microcontrollerData.encoderValues[1] - oldData.encoderValues[1];

        kinematics.integratePosition(encoderIncrement, currentPosition);

        std::cout << "\rCurrent positon: " << std::fixed << currentPosition << std::flush;
    }

}

int main(int argc, char **argv)
{
    signal(SIGINT, killCode);
    signal(SIGTERM, killCode);
    bool failed = argc < 4;
    miam::RobotPosition startPos;
    if (!failed)
    {
        try
        {
            startPos.x = std::stof(argv[1]);
            startPos.y = std::stof(argv[2]);
            startPos.theta = std::stof(argv[3]);
        }
        catch (const std::invalid_argument &)
        {
            failed = true;
        }
    }
    if (failed)
    {
        std::cout << "Usage: ./TestMotion <starx> <starty> <startz>" << std::endl;
        exit(0);
    }


    if (!uCListener_start("/dev/arduinoUno"))
    {
        std::cout << "Failed to talk to arduino, exiting." << std::endl;
        exit(0);
    }
    int maxSpeed = robotdimensions::maxWheelSpeed / robotdimensions::wheelRadius / robotdimensions::stepSize;
    int maxAcceleration = robotdimensions::maxWheelAcceleration / robotdimensions::wheelRadius / robotdimensions::stepSize;

    stepperMotors = miam::L6470("/dev/spidev0.0", 2);

    const int MOTOR_KVAL_HOLD = 0x30;
    const int MOTOR_BEMF[4] = {0x3B, 0x1430, 0x22, 0x53};
    bool isInit = stepperMotors.init(maxSpeed, maxAcceleration, MOTOR_KVAL_HOLD,
                                            MOTOR_BEMF[0], MOTOR_BEMF[1], MOTOR_BEMF[2], MOTOR_BEMF[3]);
    if (!isInit)
    {
        std::cout << "Failed to talk to stepper motors, exiting." << std::endl;
        exit(0);
    }
    stepperMotors.setStepMode(miam::L6470_STEP_MODE::MICRO_128);

    std::thread odomThread(odometry, startPos);
    odomThread.detach();

    // Init window
    Display *display;
    Window window;
    XEvent event;

    /* open connection with the server */
    display = XOpenDisplay(NULL);
    if (display == NULL)
    {
        fprintf(stderr, "Cannot open display\n");
        exit(1);
    }

    int scr = DefaultScreen(display);

    /* create window */
    window = XCreateSimpleWindow(display, RootWindow(display, scr), 10, 10, 200, 200, 1,
                           BlackPixel(display, scr), WhitePixel(display, scr));

    /* select kind of events we are interested in */
    XSelectInput(display, window, KeyPressMask | KeyReleaseMask );

    /* map (show) the window */
    XMapWindow(display, window);

    /* event loop */

    int const RELEVANT_KEYS[8] = {79, 80, 81, 83, 85, 87, 88, 89};
    double const s = maxSpeed * 0.75;
    std::vector<std::vector<double>> VELOCITIES =
        {
            {s, 0}, {s, s}, {0, s},
            {s, -s},        {-s, s},
            {-s, 0}, {-s, -s}, {0, -s}};

    std::vector<double> ZERO = {0, 0};

    // stepperMotors_.setSpeed(ZERO);

    while (true)
    {
        // Handle user input.
        XNextEvent(display, &event);

        /* exit on ESC key press */
        if (event.type == KeyPress)
            if ( event.xkey.keycode == 0x09 )
                break;
        int relevant = -1;
        for (int i = 0; i < 9; i++)
        {
            if (event.xkey.keycode == RELEVANT_KEYS[i])
            {
                relevant = i;
                break;
            }
        }
        if (relevant > -1)
        {
            if (event.type == KeyPress)
                stepperMotors.setSpeed(VELOCITIES[relevant]);
            else if (event.type == KeyRelease)
                stepperMotors.setSpeed(ZERO);
        }
    }

    /* close connection to server */
    XCloseDisplay(display);

    stepperMotors.hardStop();
    stepperMotors.highZ();

    return 0;
}
