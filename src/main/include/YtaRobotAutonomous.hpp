////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous.hpp
/// @author David Stalter
///
/// @details
/// Contains the declarations for the autonomous portions of code ran in an FRC
/// robot.
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTAROBOTAUTONOMOUS_HPP
#define YTAROBOTAUTONOMOUS_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/WPILib.h"             // for FRC library

// C++ INCLUDES
#include "YtaRobot.hpp"             // for inline autonomous function declarations

using namespace frc;

////////////////////////////////////////////////////////////////
/// @namespace YtaRobotAutonomous
///
/// Namespace that contains robot autonomous variable and
/// function declarations.
///
////////////////////////////////////////////////////////////////
namespace YtaRobotAutonomous
{
//public:
    // TYPEDEFS
    // (none)
    
    // ENUMS
    // (none)    
    
    // STRUCTS
    // (none)
    
    // MEMBER VARIABLES
    
    // CONSTS
    
    // Autonomous Mode Constants
    // TODO 2019: Use frc::SendableChooser<std::string>
    // Note: Only enable one autonomous routine!
    // Note: Autonomous routines are currently NOT controlled
    // by physical switches on the robot.
    //static const bool       ROUTINE_1                           = true;
    //static const bool       ROUTINE_2                           = false;
    //static const bool       ROUTINE_3                           = false;
    //static const bool       TEST_ENABLED                        = false;

    // Autonomous drive speed constants
    static constexpr double DRIVE_SPEED_SLOW                    =  0.30;
    static constexpr double DRIVE_SPEED_FAST                    =  0.50;
    static constexpr double TURN_SPEED                          =  0.25;
    static constexpr double COUNTERACT_COAST_MOTOR_SPEED        =  0.20;
    
    // Autonomous angle constants
    static constexpr double FORTY_FIVE_DEGREE_TURN_ANGLE        = 45.00;
    static constexpr double NINETY_DEGREE_TURN_ANGLE            = 90.00;
    static constexpr double ONE_EIGHTY_DEGREE_TURN_ANGLE        = 180.0;
    
    // Autonomous delay constants
    static constexpr double COUNTERACT_COAST_TIME_S             =  0.25;
    static constexpr double ENCODER_DRIVE_MAX_DELAY_S           =  5.00;
    static constexpr double DELAY_SHORT_S                       =  0.50;
    static constexpr double DELAY_MEDIUM_S                      =  1.00;
    static constexpr double DELAY_LONG_S                        =  2.00;
    
    // Autonomous encoder drive constants
    static const int        ENCODER_DRIVE_STRAIGHT_IN           =  12*8;                    
    static constexpr double ENCODER_COMPENSATE_SPEED            =  0.02;
    
    // Autonomous sonar drive constants
    static const int        SONAR_LATERAL_DRIVE_DIST_INCHES     =  7*12;
    static const int        SONAR_SIDE_DRIVE_DIST_INCHES        =     6;
    static const int        SONAR_MIN_DRIVE_ENABLE_INCHES       = 10*12;
    static const int        SONAR_INIT_TURN_DIST_INCHES         =     5;
    static const int        SONAR_MAX_ALLOWED_READING_DIFF      =     2;
    static const unsigned   SONAR_BUMPER_CLEARANCE_DIST_INCHES  =     4;
    static constexpr double SONAR_ROUTINE_TIME_S                =  5.00;
    static constexpr double SONAR_DRIVE_LEFT_SPEED              = -0.10;
    static constexpr double SONAR_DRIVE_RIGHT_SPEED             =  0.10;
    static constexpr double SONAR_COMPENSATE_LEFT_SPEED         = -0.05;
    static constexpr double SONAR_COMPENSATE_RIGHT_SPEED        =  0.05;
    
} // End namespace



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousDelay
///
/// Waits for a specified amount of time in autonomous.  Used
/// while an operation is ongoing but not yet complete, and
/// nothing else needs to occur.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::AutonomousDelay(double time)
{
    m_pAutonomousTimer->Start();
    while (m_pAutonomousTimer->Get() < time) {}
    m_pAutonomousTimer->Stop();
    m_pAutonomousTimer->Reset();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousDriveSequence
///
/// Drives during autonomous for a specified amount of time.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::AutonomousDriveSequence(double speed, double time)
{
    // 20xx: Left forward is positive, Right forward is negative
    
    // First turn the motors on
    m_pLeftDriveMotors->Set(speed);
    m_pRightDriveMotors->Set(-speed);

    // Time it
    AutonomousDelay(time);

    // Motors back off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousBackDrive
///
/// Back drives the motors to abruptly stop the robot.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::AutonomousBackDrive(EncoderDirection currentRobotDirection)
{
    double leftSpeed = YtaRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;
    double rightSpeed = YtaRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;
    switch (currentRobotDirection)
    {
        // 20xx: If we are currently going forward, left motor back drive is negative
        case FORWARD:
        {
            leftSpeed *= -1.0;
            break;
        }
        // 20xx: If we are currently going backward, right motor back drive is negative
        case REVERSE:
        {
            rightSpeed *= -1.0;
            break;
        }
        default:
        {
            break;
        }
    }
    
    // Counteract coast
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);
    
    // Delay
    AutonomousDelay(YtaRobotAutonomous::COUNTERACT_COAST_TIME_S);
    
    // Motors off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    m_pSafetyTimer->Reset();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::AutonomousBackDriveTurn
///
/// Back drives the motors to abruptly stop the robot during
/// a turn.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::AutonomousBackDriveTurn(GyroDirection currentGyroDirection)
{
    double leftSpeed = YtaRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;
    double rightSpeed = YtaRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;
    
    // 20xx: Left turns have -/- to the motors
    // 20xx: Right turns have +/+ to the motors
    
    // If the turn is right, counteract is -/-
    if (currentGyroDirection == RIGHT_TURN)
    {
        leftSpeed *= -1.0;
        rightSpeed *= -1.0;
    }
    
    // Counteract coast
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);
    
    // Delay
    AutonomousDelay(YtaRobotAutonomous::COUNTERACT_COAST_TIME_S);
    
    // Motors off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    m_pSafetyTimer->Reset();
}

#endif // YTAROBOTAUTONOMOUS_HPP
