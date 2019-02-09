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
I2C         RobotI2c::m_pI2cRioduino(I2C::Port::kMXP, RoborioRioduinoSharedData::I2C_DEVICE_ADDRESS);
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
    while (true) {}
    
    // Main loop
    while (true)
    {
        UpdateI2cData();
        UnpackI2cData();
        
        // Experiment with sleeping and how often things run
        auto start = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end-start;
        std::cout << "Waited " << elapsed.count() << " ms\n";
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
                uint16_t & rRobotAngle = m_I2cRioduinoData.m_DataBuffer.m_GyroData.m_xAxisInfo.m_Angle;
                
                // Make sure a valid angle came over
                if (rRobotAngle > RoborioRioduinoSharedData::GyroI2cData::MAX_VALID_ANGLE_VALUE)
                {
                    RobotUtils::DisplayMessage("Invalid angle received in I2C transfer.");
                    rRobotAngle = 0;
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
        RobotUtils::DisplayMessage("Invalid I2C metadata.");
    }
}
