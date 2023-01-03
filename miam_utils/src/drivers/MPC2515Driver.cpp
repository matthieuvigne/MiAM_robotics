/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/drivers/MCP2515Driver.h"

#include <errno.h>
#include <unistd.h>
#include <cmath>

#include <iostream>
#include <cstring>
#include <cmath>


//··································································································
//   MCP2515 COMMANDS
//··································································································

static const uint8_t RESET_COMMAND = 0xC0 ;
static const uint8_t WRITE_COMMAND = 0x02 ;
static const uint8_t READ_COMMAND  = 0x03 ;
static const uint8_t BIT_MODIFY_COMMAND         = 0x05 ;
static const uint8_t LOAD_TX_BUFFER_COMMAND     = 0x44 ;
static const uint8_t REQUEST_TO_SEND_COMMAND    = 0x84 ;
static const uint8_t READ_FROM_RXB0SIDH_COMMAND = 0x90 ;
static const uint8_t READ_FROM_RXB1SIDH_COMMAND = 0x94 ;
static const uint8_t READ_STATUS_COMMAND        = 0xA0 ;
static const uint8_t RX_STATUS_COMMAND          = 0xB0 ;

//··································································································
//   MCP2515 REGISTERS
//··································································································

static const uint8_t BFPCTRL_REGISTER   = 0x0C ;
static const uint8_t TXRTSCTRL_REGISTER = 0x0D ;
static const uint8_t CANSTAT_REGISTER   = 0x0E ;
static const uint8_t CANCTRL_REGISTER   = 0x0F ;
static const uint8_t TEC_REGISTER       = 0x1C ;
static const uint8_t REC_REGISTER       = 0x1D ;
static const uint8_t RXM0SIDH_REGISTER  = 0x20 ;
static const uint8_t RXM1SIDH_REGISTER  = 0x24 ;
static const uint8_t CNF3_REGISTER      = 0x28 ;
static const uint8_t CNF2_REGISTER      = 0x29 ;
static const uint8_t CNF1_REGISTER      = 0x2A ;
static const uint8_t CANINTE_REGISTER   = 0x2B ;
static const uint8_t CANINTF_REGISTER   = 0x2C ;
static const uint8_t EFLG_REGISTER      = 0x2D ;
static const uint8_t TXB0CTRL_REGISTER  = 0x30 ;
static const uint8_t TXB1CTRL_REGISTER  = 0x40 ;
static const uint8_t TXB2CTRL_REGISTER  = 0x50 ;
static const uint8_t RXB0CTRL_REGISTER  = 0x60 ;
static const uint8_t RXB1CTRL_REGISTER  = 0x70 ;

static const uint8_t RXFSIDH_REGISTER [6] = {0x00, 0x04, 0x08, 0x10, 0x14, 0x18} ;



MCP2515::MCP2515():
    spiDriver_(nullptr)
{

}


MCP2515::MCP2515(SPIWrapper *spiDriver):
    spiDriver_(spiDriver)
{

}

bool MCP2515::writeRegister(uint8_t const& address, uint8_t const& value)
{
    uint8_t data[3] = {WRITE_COMMAND, address, value};
    return spiDriver_->spiReadWriteSingle(data, 3) == 3;
}

uint8_t MCP2515::readRegister(uint8_t const& address)
{
    uint8_t data[3] = {READ_COMMAND, address, 0};
    if (spiDriver_->spiReadWriteSingle(data, 3) == 3)
        return data[2];
    return 0;
}


bool MCP2515::init()
{
    // Soft reset
    uint8_t data[1] = {RESET_COMMAND};
    spiDriver_->spiReadWriteSingle(data, 1);
    usleep(50);

    // Test connection
    writeRegister(CNF1_REGISTER, 0x55);
    if (readRegister(CNF1_REGISTER) != 0x55)
        return false;
    writeRegister(CNF1_REGISTER, 0x42);
    if (readRegister(CNF1_REGISTER) != 0x42)
        return false;

    // Communication frequency
    // This is hard-coded: we assume a oscillator frequency of 16MHz,
    // and a communication bitrate of 1MBps.

    // 1-bit time = 1000ns
    // Clock bit time: BRP = 0
    // TQ = 1 / 8MHz = 125ns
    // So 1bit = 8TQ
    // Setting Tdelay = 2TQ and considering 1TQ sync, we have 5TQ available
    // We set PSI = 2, PS2 = 3 and SWJ = 1
    unsigned char const BRP = 0;
    unsigned char const DELAY = 2;
    unsigned char const PS1 = 2;
    unsigned char const PS2 = 3;
    unsigned char const SWJ = 1;

    // Config
    writeRegister(CNF1_REGISTER, ((SWJ -1) << 6) + BRP);
    writeRegister(CNF2_REGISTER, 0x80 + ((PS1 - 1) << 3) + (DELAY - 1));
    writeRegister(CNF3_REGISTER, PS2 - 1);

    // Interrupts not used
    writeRegister(CANINTE_REGISTER, 0b11110000);
    writeRegister(CANINTF_REGISTER, 0x00);

    // Disable RXnBF, set TXnRTS as inputs
    writeRegister(BFPCTRL_REGISTER, 0);
    writeRegister(TXRTSCTRL_REGISTER, 0);

    // Don't use rollover nor filters
    writeRegister(RXB0CTRL_REGISTER, 0x60);

    // TXB2CTRL_REGISTER as highest priority
    writeRegister(TXB2CTRL_REGISTER, 11);

    // Set device to normal operation, non-one-shot
    writeRegister(CANCTRL_REGISTER, 0);
    // Wait for activation
    usleep(2500);

    return readRegister(CANCTRL_REGISTER) == 0;

}

bool MCP2515::sendMessage(CANMessage const& message)
{
    // Send data

    uint8_t data[6 + message.len];
    data[0] = LOAD_TX_BUFFER_COMMAND;
    data[1] = message.id >> 3;
    data[2] = (message.id << 5) & 0xE0 ;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = message.len;
    for (uint8_t i = 0; i < message.len; i++)
        data[6 + i] = message.data[i];
    int res = spiDriver_->spiReadWriteSingle(data, 6 + message.len);
    if (res != 6 + message.len)
        return false;

    data[0] = REQUEST_TO_SEND_COMMAND;
    res = spiDriver_->spiReadWriteSingle(data, 1);

    return res == 1;
}

bool MCP2515::isDataAvailable()
{
    uint8_t data[2] = {RX_STATUS_COMMAND, 0};
    int res = spiDriver_->spiReadWriteSingle(data, 2);
    return (data[1] & 0b01000000) > 0;
}

bool MCP2515::readAvailableMessage(CANMessage & message)
{
    // Read receive buffer
    uint8_t data[14];
    data[0] = READ_FROM_RXB0SIDH_COMMAND;
    int res = spiDriver_->spiReadWriteSingle(data, 14);
    if (res != 14)
        return false;

    message.id = (data[1] << 3) + (data[2] >> 5);
    message.len = std::min(data[5] & 0x0F, 8);

    for (uint8_t i = 0; i < message.len; i++)
        message.data[i] = data[6 + i];

    // Free recieve buffer.
    data[0] = BIT_MODIFY_COMMAND;
    data[1] = CANINTF_REGISTER;
    data[2] = 0x01;
    data[3] = 0;
    res = spiDriver_->spiReadWriteSingle(data, 4);

    // std::cout << "CAN read:";
    // for (int i = 0; i < 8; i++)
    //     std::cout << int(message.data[i]) << " ";
    // std::cout << std::endl;


    return res == 4;
}

