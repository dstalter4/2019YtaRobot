////////////////////////////////////////////////////////////////////////////////
/// @file   RobotI2c.cpp
/// @author David Stalter
///
/// @details
/// Contains function definitions for interacting with and controlling I2C on
/// the robot.
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "RobotI2c.hpp"                         // for class declaration
#include "RobotUtils.hpp"                       // for DisplayMessage()
#include "YtaRobot.hpp"                         // for I2cSequence() declaration

// STATIC MEMBER DATA
DigitalOutput           RobotI2c::m_DigitalOutputToRioduino(ROBORIO_SIGNAL_DIO_PIN);
DigitalInput            RobotI2c::m_DigitalInputFromRioduino(RIODUINO_SIGNAL_DIO_PIN);
I2cCommand              RobotI2c::m_I2cRioduinoCommand;
I2cData                 RobotI2c::m_I2cRioduinoData;
I2C                     RobotI2c::m_I2cRioduino(I2C::Port::kMXP, RoborioRioduinoSharedData::I2C_DEVICE_ADDRESS);
bool                    RobotI2c::m_bI2cDataValid           = false;
bool                    RobotI2c::m_bI2cCommandReady        = false;
RobotI2c::ThreadPhase   RobotI2c::m_ThreadPhase             = TRIGGER_INTERRUPT;
unsigned int            RobotI2c::m_ThreadUpdateRateMs      = DEFAULT_UPDATE_RATE_MS;
unsigned int            RobotI2c::m_NumValidTransactions    = 0U;
unsigned int            RobotI2c::m_NumInvalidTransactions  = 0U;



////////////////////////////////////////////////////////////////
/// @method RobotI2c::I2cThread
///
/// The main I2C thread on the robot.
///
////////////////////////////////////////////////////////////////
void RobotI2c::I2cThread()
{
    RobotUtils::DisplayMessage("I2C thread detached.");

    // The RIOduino will have booted well before this.
    // Trigger both of its loop control variables to make
    // sure it doesn't hang during the main loop.
    m_DigitalOutputToRioduino.Set(true);
    UpdateI2cData();
    std::this_thread::sleep_for(std::chrono::milliseconds(INITIALIZING_DELAY_MS));
    m_DigitalOutputToRioduino.Set(false);
    
    while (true)
    {
        switch (m_ThreadPhase)
        {
            case TRIGGER_INTERRUPT:
            {
                // Trigger the interrupt
                m_DigitalOutputToRioduino.Set(true);
                m_ThreadPhase = COLLECT_DATA;
                break;
            }
            case COLLECT_DATA:
            {
                // Wait for an indication the data is ready
                if (m_DigitalInputFromRioduino.Get())
                {
                    // Get and process new I2C data
                    UpdateI2cData();
                    UnpackI2cData();
                    
                    // Clear the interrupt trigger
                    m_DigitalOutputToRioduino.Set(false);
                    
                    m_ThreadPhase = DELAY;
                }
                break;
            }
            case SEND_COMMAND:
            {
                // Send the I2C command
                SendI2cCommand();
                
                // A command is no longer ready
                m_bI2cCommandReady = false;
                
                m_ThreadPhase = DELAY;
                break;
            }
            case DELAY:
            {
                // Relinquish the CPU
                std::this_thread::sleep_for(std::chrono::milliseconds(m_ThreadUpdateRateMs));
                
                // Check if a request to send a command came in (could be external to this thread)
                if (m_bI2cCommandReady)
                {
                    m_ThreadPhase = SEND_COMMAND;
                }
                else
                {
                    m_ThreadPhase = TRIGGER_INTERRUPT;
                }
                
                break;
            }
            default:
            {
                break;
            }
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method RobotI2c::SendCommand
///
/// Sends a command to the RIOduino via I2C.
///
////////////////////////////////////////////////////////////////
void RobotI2c::SendCommand(I2cCommandSelection command)
{
    // First clear the buffer
    std::memset(&m_I2cRioduinoCommand, I2C_BUFFER_MARKER, sizeof(m_I2cRioduinoCommand));
    
    // Build the metadata.  
    // This will only retain the last command that is sent, so some could be dropped.
    m_I2cRioduinoCommand.m_Header = I2C_HEADER_DATA;
    m_I2cRioduinoCommand.m_Footer = I2C_FOOTER_DATA;
    m_I2cRioduinoCommand.m_CommandSelection = command;
    
    m_bI2cCommandReady = true;
}



////////////////////////////////////////////////////////////////
/// @method RobotI2c::UnpackI2cData
///
/// Upacks received I2C data.
///
////////////////////////////////////////////////////////////////
void RobotI2c::UnpackI2cData()
{
    m_bI2cDataValid = false;
        
    // Make sure a valid data packet was received
    if ((m_I2cRioduinoData.m_Header == I2C_HEADER_DATA) &&
        (m_I2cRioduinoData.m_Footer == I2C_FOOTER_DATA))
    {
        // The unpacking action depends on what kind of data was sent
        switch (m_I2cRioduinoData.m_DataSelection)
        {
            // Currently only supporting gyro data
            case I2cDataSelection::GYRO_DATA:
            {
                // Read the angle
                uint16_t * pRobotAngle = &m_I2cRioduinoData.m_DataBuffer.m_GyroData.m_xAxisInfo.m_Angle;
                
                // Make sure a valid angle came over
                if (*pRobotAngle > RoborioRioduinoSharedData::GyroI2cData::MAX_VALID_ANGLE_VALUE)
                {
                    RobotUtils::DisplayMessage("Invalid angle received in I2C transfer.");
                    *pRobotAngle = 0;
                }
                
                break;
            }
            default:
            {
                // Do nothing in case a bad packet is received
                break;
            }
        }
        
        m_NumValidTransactions++;
        m_bI2cDataValid = true;
    }
    else
    {
        m_NumInvalidTransactions++;
        
        if (DEBUG_I2C_TRANSACTIONS)
        {
            RobotUtils::DisplayMessage("Invalid I2C metadata.");
            RobotUtils::DisplayFormattedMessage("Transactions - Valid: %u, Invalid: %u\n", m_NumValidTransactions, m_NumInvalidTransactions);
            RobotUtils::DisplayMessage("Dumping buffer...");
            
            uint8_t * pData = reinterpret_cast<uint8_t *>(&m_I2cRioduinoData);
            for (int i = 0; i < sizeof(m_I2cRioduinoData); i++)
            {
                RobotUtils::DisplayFormattedMessage("%x ", *pData++);
            }
            RobotUtils::DisplayMessage("\n");
        }
    }
}
