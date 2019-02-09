////////////////////////////////////////////////////////////////////////////////
/// @file   RobotI2c.hpp
/// @author David Stalter
///
/// @details
/// Contains declarations for interacting with and controlling I2C on the robot.
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef ROBOTI2C_HPP
#define ROBOTI2C_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/WPILib.h"                                     // for FRC library

// C++ INCLUDES
#include "RobotUtils.hpp"                                   // for DisplayMessage()
#include "../../Rioduino/RoborioRioduinoSharedData.hpp"     // for shared data structures

using namespace frc;
using namespace RoborioRioduinoSharedData;


////////////////////////////////////////////////////////////////
/// @class RobotI2c
///
/// Class that provides methods for interacting with I2C.
///
////////////////////////////////////////////////////////////////
class RobotI2c
{
public:
    
    // The vision thread
    static void I2cThread();
    
    // Set the rate for how fast the thread should run
    inline static void SetThreadUpdateRate(unsigned updateRateMs);
    
    // Retrieve the I2C sonar data
    inline static SonarI2cData * GetSonarData();
    
    // Retrieve the I2C gyro data
    inline static GyroI2cData * GetGyroData();

private:
    
    // Autonomous control flow
    static void AutonomousI2cSequence();
    
    // Teleop control flow
    static void TeleopI2cSequence();
    
    // Unpack the received I2C data
    static void UnpackI2cData();
    
    // Update the I2c data structures
    inline static void UpdateI2cData();

    // Constructor
    RobotI2c();

    // Destructor, copy constructor, assignment operator
    ~RobotI2c();

    RobotI2c(const RobotI2c &) = delete;
    RobotI2c & operator=(const RobotI2c &) = delete;
    
    // MEMBER VARIABLES
    
    // I2C configuration
    static I2cData      m_I2cRioduinoData;
    static I2C          m_pI2cRioduino;
    static bool         m_bI2cDataValid;
    static unsigned int m_ThreadUpdateRateMs;
    
    static const unsigned DEFAULT_UPDATE_RATE_MS = 10U;
};



////////////////////////////////////////////////////////////////
/// @method RobotI2c::SetThreadUpdateRate
///
/// Sets how fast the I2C thread should get new data from the
/// RIOduino.
///
////////////////////////////////////////////////////////////////
void RobotI2c::SetThreadUpdateRate(unsigned updateRateMs)
{
    m_ThreadUpdateRateMs = updateRateMs;
}



////////////////////////////////////////////////////////////////
/// @method RobotI2c::UpdateI2cData
///
/// Gets new I2C data from the RIOduino.
///
////////////////////////////////////////////////////////////////
inline void RobotI2c::UpdateI2cData()
{
    // Clear the buffer for new data
    std::memset(&m_I2cRioduinoData, 0U, sizeof(m_I2cRioduinoData));
    
    // Get the data from the riodiuino
    //uint8_t I2C_WRITE_STRING[] = "Frc120I2c";
    //static_cast<void>(m_pI2cRioduino.WriteBulk(&I2C_WRITE_STRING[0], sizeof(I2C_WRITE_STRING)));
    static_cast<void>(m_pI2cRioduino.ReadOnly(sizeof(m_I2cRioduinoData), reinterpret_cast<uint8_t *>(&m_I2cRioduinoData)));
    //static_cast<void>(m_pI2cRioduino.Transaction(I2C_STRING, sizeof(I2C_STRING), reinterpret_cast<uint8_t *>(&m_I2cRioduinoData), sizeof(m_I2cRioduinoData)));
}



////////////////////////////////////////////////////////////////
/// @method RobotI2c::GetSonarData
///
/// Returns the I2C sonar data from the RIOduino.
///
////////////////////////////////////////////////////////////////
inline SonarI2cData * RobotI2c::GetSonarData()
{
    if (m_I2cRioduinoData.m_DataSelection != I2cDataSelection::SONAR_DATA)
    {
        RobotUtils::DisplayMessage("Received data was not selected as sonar, could be invalid!");
    }
    
    return &m_I2cRioduinoData.m_DataBuffer.m_SonarData;
}



////////////////////////////////////////////////////////////////
/// @method RobotI2c::GetGyroData
///
/// Returns the I2C gyro data from the RIOduino.
///
////////////////////////////////////////////////////////////////
inline GyroI2cData * RobotI2c::GetGyroData()
{
    if (m_I2cRioduinoData.m_DataSelection != I2cDataSelection::GYRO_DATA)
    {
        RobotUtils::DisplayMessage("Received data was not selected as gyro, could be invalid!");
    }
    
    return &m_I2cRioduinoData.m_DataBuffer.m_GyroData;
}

#endif // ROBOTI2C_HPP
