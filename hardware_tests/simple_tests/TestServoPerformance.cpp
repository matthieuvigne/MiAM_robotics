/// A simple test for the AS5045 encoder

#include <miam_utils/Logger.h>
#include <miam_utils/Metronome.h>
#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/drivers/BMI088Driver.h>
#include <miam_utils/drivers/PCA9546ADriver.h>
#include <miam_utils/drivers/INA226Driver.h>
#include <miam_utils/drivers/STSServoDriver.h>

#include <iostream>
#include <unistd.h>
#include <iomanip>

double const PERIOD = 1.2;

class ServoTester{
    public:
        ServoTester(STSServoDriver *driver,
            Logger *logger,
            std::string motorName,
            int servoId,
            bool isSCS = false) :
            driver_(driver),
            logger_(logger),
            motorName_(motorName),
            servoId_(servoId),
            isSCS_(isSCS)
        {
            driver_->setMode(servoId_, STS::Mode::POSITION);

        }

        void setServoPosition(int servoPos, int nServos)
        {
            servoVectorId_ = servoPos;
            nServos_ = nServos;
        }

        void update(double const& currentTime)
        {
            usleep(1000);
            double periodTime = currentTime;
            while (periodTime > 2 * PERIOD * nServos_)
                periodTime -= 2 * PERIOD * nServos_;
            double targetPosition = 0;
            if (periodTime > PERIOD * servoVectorId_ &&  periodTime < PERIOD * (servoVectorId_ + nServos_))
                targetPosition = M_PI_2;
            driver_->setTargetPosition(servoId_, posToRaw(targetPosition));
            logger_->log("Servos." + motorName_ + ".target", currentTime, targetPosition);
            logger_->log("Servos." + motorName_ + ".currentPosition", currentTime, rawToRad(driver_->getCurrentPosition(servoId_)));
            logger_->log("Servos." + motorName_ + ".currentVelocity", currentTime, rawToRads(driver_->getCurrentVelocity(servoId_)));
        }

    private:
        int posToRaw(double pos)
        {
            if (isSCS_)
                return 512 + (pos * 180.0 / M_PI) / 0.293;
            return 2048 + pos * 2048 / M_PI;
        }
        double rawToRad(int pos)
        {
            if (isSCS_)
                return (pos - 512) * 0.293 * M_PI / 180.0;
            return (pos - 2048) * M_PI / 2048.0;
        }
        double rawToRads(int vel)
        {
            if (isSCS_)
                return vel * 0.293 * M_PI / 180.0;
            return vel * M_PI / 2048.0;
        }
        STSServoDriver *driver_;
        Logger *logger_;
        std::string motorName_;
        int servoId_;
        int servoVectorId_;
        int nServos_;
        bool isSCS_;
};

int main (int argc, char *argv[])
{
    RPi_enablePorts();


    PCA9546A i2cExpander_;
    INA226 ina226_7V_;
    INA226 ina226_12V_;

    bool initSuccess = i2cExpander_.init(&RPI_I2C);
    i2cExpander_.setPorts(0xFF);
    initSuccess &= ina226_7V_.init(&RPI_I2C, 0x45, 0.010);
    initSuccess &= ina226_12V_.init(&RPI_I2C, 0x44, 0.010);


    STSServoDriver driver;
    initSuccess &= driver.init("/dev/ttyAMA0", -1);

    if (!initSuccess)
    {
        std::cout << "Init failed" << std::endl;
        return -1;
    }

    Logger logger;
    logger.start("ServoTest.data");

    std::vector<ServoTester> tester;
    tester.push_back(ServoTester(&driver, &logger, "33", 33));
    tester.push_back(ServoTester(&driver, &logger, "7", 7));
    tester.push_back(ServoTester(&driver, &logger, "12", 12));
    tester.push_back(ServoTester(&driver, &logger, "21", 21));
    // driver.writeRegister(99, STS::registers::POS_PROPORTIONAL_GAIN, 50);
    // driver.writeRegister(99, STS::registers::POS_INTEGRAL_GAIN, 50);
    // driver.writeRegister(99, STS::registers::SPEED_PROPORTIONAL_GAIN, 50);
    // driver.writeRegister(99, STS::registers::SPEED_INTEGRAL_GAIN, 50);
    // tester.push_back(ServoTester(&driver, &logger, "SCS0009", 100, true));
    // tester.push_back(ServoTester(&driver, &logger, "Waveshare", 13));
    // tester.push_back(ServoTester(&driver, &logger, "STS3045", 101));
    // tester.push_back(ServoTester(&driver, &logger, "STS3032", 102));

    for (int i = 0; i < static_cast<int>(tester.size()); i++)
        tester.at(i).setServoPosition(i, static_cast<int>(tester.size()));

    Metronome metronome(0.005 * 1e9);

    double lastTime;
    while (true)
    {
        metronome.wait();
        double const currentTime = metronome.getElapsedTime();
        for (auto& t : tester)
            t.update(currentTime);

        INA226Reading inaReading = ina226_7V_.read();
        logger.log("Robot.7V.voltage", currentTime, inaReading.voltage);
        logger.log("Robot.7V.current", currentTime, inaReading.current);
        logger.log("Robot.7V.power", currentTime, inaReading.power);
        inaReading = ina226_12V_.read();
        logger.log("Robot.12V.voltage", currentTime, inaReading.voltage);
        logger.log("Robot.12V.current", currentTime, inaReading.current);
        logger.log("Robot.12V.power", currentTime, inaReading.power);
        logger.log("timeDiff", currentTime, currentTime - lastTime);
        lastTime = currentTime;
    }

    return 0;
}


