////////////////////////////////////////////////////////////////////////////////
/// @file   AutonomousGyro.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous gyroscope routines.
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"                 // for robot class declaration
#include "YtaRobotAutonomous.hpp"       // for autonomous declarations



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousGyroLeftTurn
///
/// Turns the robot left based on gyro readings.
///
////////////////////////////////////////////////////////////////
bool YtaRobot::AutonomousGyroLeftTurn(double destAngle, double turnSpeed)
{
    // 20xx LEFT FORWARD DRIVE IS POSITIVE
    // 20xx RIGHT FORWARD DRIVE IS NEGATIVE    
    // 20xx LEFT TURNS DECREASE GYRO ANGLE
    // Left turn is left motors back, right motors forward
    
    double startAngle = GetGyroValue(BNO055);
    
    // Left turns are right motors forward, left motors reverse
    m_pLeftDriveMotors->Set(-turnSpeed);
    m_pRightDriveMotors->Set(-turnSpeed);
    
    m_pSafetyTimer->Reset();
    m_pSafetyTimer->Start();
    
    // Angle will be decreasing
    while ((GetGyroValue(BNO055) > (startAngle - destAngle)) && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE))
    {
        if (!m_pDriverStation->IsAutonomous())
        {
            break;
        }
        
        SmartDashboard::PutNumber("Gyro angle", GetGyroValue(BNO055));
    }
    
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    m_pSafetyTimer->Stop();
    if (m_pSafetyTimer->Get() > SAFETY_TIMER_MAX_VALUE)
    {
        m_pSafetyTimer->Reset();
        return false;
    }
    m_pSafetyTimer->Reset();
    
    // Counteract coast
    AutonomousBackDriveTurn(LEFT_TURN);
    
    return true;
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousGyroRightTurn
///
/// Turns the robot right based on gyro readings.
///
////////////////////////////////////////////////////////////////
bool YtaRobot::AutonomousGyroRightTurn(double destAngle, double turnSpeed)
{
    // 20xx LEFT FORWARD DRIVE IS POSITIVE
    // 20xx RIGHT FORWARD DRIVE IS NEGATIVE
    // 20xx RIGHT TURNS INCREASE GYRO ANGLE
    // Right turn is left motors forward, right motors back
    
    double startAngle = GetGyroValue(BNO055);
    
    // Right turns are left motors forward, right motors reverse
    m_pLeftDriveMotors->Set(turnSpeed);
    m_pRightDriveMotors->Set(turnSpeed);
    
    m_pSafetyTimer->Reset();
    m_pSafetyTimer->Start();
    
    // Angle will be increasing
    while ((GetGyroValue(BNO055) < (startAngle + destAngle)) && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE))
    {
        if (!m_pDriverStation->IsAutonomous())
        {
            break;
        }
        
        SmartDashboard::PutNumber("Gyro angle", GetGyroValue(BNO055));
    }
    
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    m_pSafetyTimer->Stop();
    if (m_pSafetyTimer->Get() > SAFETY_TIMER_MAX_VALUE)
    {
        m_pSafetyTimer->Reset();
        return false;
    }
    m_pSafetyTimer->Reset();
    
    // Counteract coast
    AutonomousBackDriveTurn(RIGHT_TURN);
    
    return true;
}
