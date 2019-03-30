////////////////////////////////////////////////////////////////////////////////
/// @file   YtaController.hpp
/// @author David Stalter
///
/// @details
/// A class designed to interface to several controller types (Logitech Gamepad,
/// Xbox GameSir) with custom responses.
///
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTACONTROLLER_HPP
#define YTACONTROLLER_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/WPILib.h"                 // for FRC library

// C++ INCLUDES
// (none)

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class YtaController
///
/// Class that provides methods for interacting with a Logitech
/// Gamepad controller.  Derives from GenericHID.  Inheriting
/// from GamepadBase.h is deprecated, so GenericHID is used
/// directly.
///
////////////////////////////////////////////////////////////////
class YtaController : public GenericHID
{
    friend class YtaRobot;
    
public:
    
    explicit YtaController(int port);
    virtual ~YtaController() = default;
    
    virtual double GetX(JoystickHand hand = kLeftHand) const override;
    virtual double GetY(JoystickHand hand = kLeftHand) const override;
    
    double GetThrottle() const;
    
private:
    
    // Joystick axes inputs are:
    // L2 0->1 R2
    //  -1    +1
    
    enum RawAxes
    {
        LEFT_X_AXIS         = 0,
        LEFT_Y_AXIS         = 1,
        LEFT_TRIGGER        = 2,
        RIGHT_TRIGGER       = 3,
        RIGHT_X_AXIS        = 4,
        RIGHT_Y_AXIS        = 5
    };
    
    enum RawButtons
    {
        NO_BUTTON           = 0,
        A                   = 1,
        B                   = 2,
        X                   = 3,
        Y                   = 4,
        LT                  = 5,
        RT                  = 6,
        SELECT              = 7,
        START               = 8,
        LEFT_STICK_CLICK    = 9,
        RIGHT_STICK_CLICK   = 10
    };
    
    double m_ThrottleValue;
    
    // @todo: These impact all controllers, not just driving.
    static constexpr double X_AXIS_SENSITIVITY_SCALING = 0.65;
    static constexpr double Y_AXIS_SENSITIVITY_SCALING = 0.90;
    
    // Prevent copying/assignment
    YtaController(const YtaController&) = delete;
    YtaController& operator=(const YtaController&) = delete;
};

#endif // YTACONTROLLER_HPP
