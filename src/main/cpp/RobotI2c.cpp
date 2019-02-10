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
#include <iostream>                             // for cout

// C INCLUDES
// (none)

// C++ INCLUDES
#include "RobotI2c.hpp"                         // for class declaration
#include "RobotUtils.hpp"                       // for DisplayMessage()
#include "YtaRobot.hpp"                         // for I2cSequence() declaration

// STATIC MEMBER DATA
I2cData     RobotI2c::m_I2cRioduinoData;
I2C         RobotI2c::m_I2cRioduino(I2C::Port::kMXP, RoborioRioduinoSharedData::I2C_DEVICE_ADDRESS);
bool        RobotI2c::m_bI2cDataValid = false;
unsigned    RobotI2c::m_ThreadUpdateRateMs = DEFAULT_UPDATE_RATE_MS;


////////////////////////////////////////////////////////////////
/// @method RobotI2c::I2cThread
///
/// The main I2C thread on the robot.
///
////////////////////////////////////////////////////////////////
void RobotI2c::I2cThread()
{
    RobotUtils::DisplayMessage("I2C thread detached.");
    
    // Main loop
    while (true)
    {
        // Get and process new I2C data
        UpdateI2cData();
        UnpackI2cData();
        
        // Sleep for a bit to not flood the RIOduino with transactions
        std::this_thread::sleep_for(std::chrono::milliseconds(m_ThreadUpdateRateMs));
    }
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
    if ((m_I2cRioduinoData.m_Header == I2cData::I2C_HEADER_DATA) &&
        (m_I2cRioduinoData.m_Footer == I2cData::I2C_FOOTER_DATA))
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

        m_bI2cDataValid = true;
    }
    else
    {
        if (DEBUG_I2C_TRANSACTIONS)
        {
            RobotUtils::DisplayMessage("Invalid I2C metadata.  Dumping buffer...");
            
            uint8_t * pData = reinterpret_cast<uint8_t *>(&m_I2cRioduinoData);
            for (int i = 0; i < sizeof(m_I2cRioduinoData); i++)
            {
                printf("%x ", *pData++);
            }
            printf("\n");
        }
    }
}
