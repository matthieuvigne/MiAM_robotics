/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "MiAMEurobot/drivers/ADNS9800Driver.h"
#include "MiAMEurobot/drivers/ADNS9800Firmware.h"
#include "MiAMEurobot/drivers/SPI-Wrapper.h"
#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>

// Registers
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst 0x64


// Internal functions: all functions accessible outside of this file are at the end.
void ADNS9800_write_register(ADNS9800 a, unsigned char address, unsigned char data)
{
    // Open SPI port
    a.port = spi_open(a.portName, a.frequency);

    // Set MSI of addresss to 1 to indicate a read operation.
    address = address | 0x80;

    // Send one transmission containing the two-byte write message
    unsigned char message[2] = {address, data};
    struct spi_ioc_transfer spiCtrl;
    // First element: send address and wait.
    spiCtrl.tx_buf = (unsigned long)&message;
    spiCtrl.rx_buf = (unsigned long)&message;
    spiCtrl.len = 2;
    spiCtrl.speed_hz = a.frequency;
    spiCtrl.bits_per_word = 8;
    spiCtrl.delay_usecs = 0;
    spiCtrl.cs_change = true;
    // Send the data over spi.
    int error = ioctl(a.port, SPI_IOC_MESSAGE(1), &spiCtrl);
    if(error <0)
    {
        #ifdef DEBUG
            printf("SPI error when writing: %d\n", error);
        #endif
    }
    spi_close(a.port);
    // 120us delay after read command
    usleep(120);
}

unsigned char ADNS9800_read_register(ADNS9800 a, unsigned char address)
{
    // Open SPI port
    a.port = spi_open(a.portName, a.frequency);

    // Set MSI of addresss to 0 to indicate a read operation.
    address = address & 0x7f;
    unsigned char response = 0;

    // We will send two transmissions: thie first one will send the address with a read command.
    // A delay of tsrad delay=100us is then applied, and the response is read by sending zero.
    struct spi_ioc_transfer spiCtrl[2];
    // First element: send address and wait.
    spiCtrl[0].tx_buf = (unsigned long)&address;
    spiCtrl[0].rx_buf = (unsigned long)&response;
    spiCtrl[0].len = 1;
    spiCtrl[0].speed_hz = a.frequency;
    spiCtrl[0].bits_per_word = 8;
    spiCtrl[0].delay_usecs = 150;
    spiCtrl[0].cs_change = false;
    // Second element: read the slave response.
    spiCtrl[1].tx_buf = (unsigned long)&address;
    spiCtrl[1].rx_buf = (unsigned long)&response;
    spiCtrl[1].len = 1;
    spiCtrl[1].speed_hz = a.frequency;
    spiCtrl[1].bits_per_word = 8;
    spiCtrl[1].delay_usecs = 0;
    spiCtrl[1].cs_change = true;
    // Send the data over spi.
    int error = ioctl(a.port, SPI_IOC_MESSAGE(2), &spiCtrl);
    if(error <0)
    {
        #ifdef DEBUG
            printf("SPI error when reading: %d\n", error);
        #endif
    }
    spi_close(a.port);
    // 20us delay after read command
    usleep(20);
    return response;
}

bool ADNS9800_write_firmware(ADNS9800 a)
{
    // Write firmware over SPI, cf p.18 of the datasheet

    // Setup SROM writting
    ADNS9800_write_register(a, REG_Configuration_IV, 0x02);
    ADNS9800_write_register(a, REG_SROM_Enable, 0x1d);

    // Wait for more than one frame period
    usleep(10000);

    // Write 0x18 to SROM_enable to start SROM download
    ADNS9800_write_register(a, REG_SROM_Enable, 0x18);

    // Load SROM file
    unsigned char message[firmware_length + 1];
    message[0] = REG_SROM_Load_Burst | 0x80;
    for(int x = 0; x < firmware_length; x++)
        message[x+1] = firmware_data[x];

    // Write it in one SPI transaction, starting at address REG_SROM_Load_Burst.
    a.port = spi_open(a.portName, a.frequency);
    struct spi_ioc_transfer spiCtrl;
    // First element: send address and wait.
    spiCtrl.tx_buf = (unsigned long)&message;
    spiCtrl.rx_buf = (unsigned long)&message;
    spiCtrl.len = firmware_length + 1;
    spiCtrl.speed_hz = a.frequency;
    spiCtrl.bits_per_word = 8;
    spiCtrl.delay_usecs = 15;
    spiCtrl.cs_change = false;
    // Send the data over spi.
    int error = ioctl(a.port, SPI_IOC_MESSAGE(1), &spiCtrl);
    if(error <0)
    {
        #ifdef DEBUG
            printf("SPI error when reading: %d\n", error);
        #endif
    }
    spi_close(a.port);
    // Sleep 160us for SROM reboot
    usleep(160);

    // Check that firmware ID is not 0
    if(ADNS9800_read_register(a, REG_SROM_ID) == 0)
    {
        #ifdef DEBUG
            printf("Error writting SROM firmware.\n");
        #endif
        return false;
    }
    return true;
}

void ADNS9800_getMotionCounts(ADNS9800 a, int *deltaX, int *deltaY)
{
    // Reset deltaX and deltaY
    *deltaX = 0;
    *deltaY = 0;
    // Read the motion register : if MSB is 0, no motion occured.
    if((ADNS9800_read_register(a, REG_Motion) & (1 << 7)) == 0)
        return;
    // Otherwise, read the new increment
    unsigned char x_L = ADNS9800_read_register(a, REG_Delta_X_L);
    unsigned char x_H = ADNS9800_read_register(a, REG_Delta_X_H);
    unsigned char y_L = ADNS9800_read_register(a, REG_Delta_Y_L);
    unsigned char y_H = ADNS9800_read_register(a, REG_Delta_Y_H);
    // Recreate full number
    *deltaX = (x_H << 8) + x_L;
    *deltaY = (y_H << 8) + y_L;
    // Get signed value: the value in the registers are 2's complements.
    // If MSB is 1, the number is negative: flip it and add one.
    if((*deltaX & (1 << 15)) > 0)
        *deltaX = *deltaX | ~((1 << 16) - 1);
    if((*deltaY & (1 << 15)) > 0)
        *deltaY = *deltaY | ~((1 << 16) - 1);
}

void ADNS9800_getMotion(ADNS9800 a, double *deltaX, double *deltaY)
{
    // Get motion in counts.
    int dx, dy;
    ADNS9800_getMotionCounts(a, &dx, &dy);
    // Convert counts to mm : TODO

    *deltaX = a.resolution * dx;
    *deltaY = a.resolution* dy;
}

bool ANDS9800_init(ADNS9800 *a, std::string const& portName)
{
    a->portName = portName;
    // Set bus frequency: default 800kHz
    a->frequency = 800000;

    // Check that communication is working by reading the REG_Product_ID register
    if(ADNS9800_read_register(*a, REG_Product_ID) != 0x33)
    {
        #ifdef DEBUG
            printf("Error reading data from ADNS9800.\n");
        #endif
        return false;
    }

    //Reste ADNS9800, see datasheet p21
    ADNS9800_write_register(*a, REG_Power_Up_Reset, 0x5a);
    usleep(50000);
    ADNS9800_write_register(*a, REG_Observation, 0x00);
    usleep(10000);
    ADNS9800_read_register(*a, REG_Observation);
    ADNS9800_read_register(*a, REG_Motion);
    ADNS9800_read_register(*a, REG_Delta_X_L);
    ADNS9800_read_register(*a, REG_Delta_X_H);
    ADNS9800_read_register(*a, REG_Delta_Y_L);
    ADNS9800_read_register(*a, REG_Delta_Y_H);

    // upload the firmware
    if(ADNS9800_write_firmware(*a) == false)
        return false;

    //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
    // reading the actual value of the register is important because the real
    // default value is different from what is said in the datasheet, and if you
    // change the reserved bytes (like by writing 0x00...) it would not work.
    unsigned char laser_ctrl0 = ADNS9800_read_register(*a, REG_LASER_CTRL0);
    ADNS9800_write_register(*a, REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );
    ADNS9800_write_register(*a, 0x2E, 0b00011111 );

    // Compute sensor resolution: 200dpi * REG_Configuration_I.
    a->resolution = 25.4 / (200.0 * (ADNS9800_read_register(*a, REG_Configuration_I) & 0b00111111));
    return true;
}
