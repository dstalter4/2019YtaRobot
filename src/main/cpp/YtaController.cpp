////////////////////////////////////////////////////////////////////////////////
/// @file   YtaController.cpp
/// @author David Stalter
///
/// @details
/// A class designed to interface to several controller types (Logitech Gamepad,
/// Xbox GameSir) with custom responses.
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaController.hpp"        // for class declaration

// STATIC MEMBER DATA
// (none)



////////////////////////////////////////////////////////////////
/// @method YtaController::YtaController
///
/// Constructor.
///
////////////////////////////////////////////////////////////////
YtaController::YtaController(int port)
: GenericHID(port)
, m_ThrottleValue(0.0)
{
}



////////////////////////////////////////////////////////////////
/// @method YtaController::GetX
///
/// Returns x-axis input.  This method is pure virtual in the
/// base class and must be implemented.
///
////////////////////////////////////////////////////////////////
double YtaController::GetX(JoystickHand hand) const
{
    // x-axis controls are very sensitive on this
    // controller, so scale them back.
    return (GetRawAxis(LEFT_X_AXIS) * X_AXIS_SENSITIVITY_SCALING);
}



////////////////////////////////////////////////////////////////
/// @method YtaController::GetY
///
/// Returns y-axis input.  This method is pure virtual in the
/// base class and must be implemented.
///
////////////////////////////////////////////////////////////////
double YtaController::GetY(JoystickHand hand) const
{
    // In order to keep the drive logic the same as the
    // joysticks, full forward is represented by -1 and
    // full reverse is represented by +1.
    
    // Left trigger is the 'reverse' value input.
    double leftTriggerValue = GetRawAxis(LEFT_TRIGGER);
    
    // Right trigger is the 'forward' value input, which
    // needs to be negated.
    double rightTriggerValue = -GetRawAxis(RIGHT_TRIGGER);
    
    // Hopefully only one trigger is being pushed, but in
    // case both are being pressed, the value will be combined.
    return (leftTriggerValue + rightTriggerValue) * Y_AXIS_SENSITIVITY_SCALING;
}



////////////////////////////////////////////////////////////////
/// @method YtaController::GetThrottle
///
/// Returns throttle control.  Most controllers do not have an
/// axis that retains its position when not being manipulated by
/// the user.  This requires throttle control to be implemented
/// and remembered in software.
///
////////////////////////////////////////////////////////////////
double YtaController::GetThrottle() const
{
    // Not implemented yet
    return 1.0;
}
