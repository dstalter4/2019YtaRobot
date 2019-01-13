////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobotAutonomous2.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 2 for YtaRobot.
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
/// @method YtaRobot::AutonomousRoutine2
///
/// Autonomous routine 2.
///
////////////////////////////////////////////////////////////////
void YtaRobot::AutonomousRoutine2()
{
    // Idle until auto is terminated
    DisplayMessage("Auto routine 2 done.");
    while ( m_pDriverStation->IsAutonomous() && m_pDriverStation->IsEnabled() )
    {
    }
}
