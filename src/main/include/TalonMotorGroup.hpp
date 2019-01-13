////////////////////////////////////////////////////////////////////////////////
/// @file   TalonMotorGroup.hpp
/// @author David Stalter
///
/// @details
/// A class designed to work with a group of CAN Talon speed controllers working
/// in tandem.
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef TALONMOTORGROUP_HPP
#define TALONMOTORGROUP_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "ctre/Phoenix.h"               // for CTRE library interfaces

// C++ INCLUDES
// (none)


////////////////////////////////////////////////////////////////
/// @class TalonMotorGroup
///
/// Class that provides methods for interacting with a group of
/// Talon speed controllers.
///
////////////////////////////////////////////////////////////////
class TalonMotorGroup
{
public:
    
    // Represents how a motor will be controlled
    enum MotorGroupControlMode
    {
        MASTER,                 // First motor in a group
        FOLLOW,                 // Motor follows the master
        INDEPENDENT,            // Motor needs to be set independently
        INVERSE,                // Motor is the inverse value of the master
        INDEPENDENT_OFFSET,     // Motor is set independently, but with a different value from master
        INVERSE_OFFSET,         // Motor is set independently, but with the a different inverse value from master
        CUSTOM                  // Motor needs to be set later to an option above
    };

    // Constructor
    TalonMotorGroup(
                     int numMotors,
                     int masterCanId,
                     MotorGroupControlMode nonMasterControlMode,
                     FeedbackDevice sensor = static_cast<FeedbackDevice>(FEEDBACK_DEVICE_NONE)
                   );

    // Adds a new motor to a group
    bool AddMotorToGroup(MotorGroupControlMode controlMode);
    
    // Function to set the speed of each motor in the group
    void Set( double value, double offset = 0.0 );
    
    // Sets the control mode of a motor in a group (intended for use with the CUSTOM group control mode)
    bool SetMotorInGroupControlMode(int canId, MotorGroupControlMode controlMode);
    
    // Change Talon mode between brake/coast
    void SetCoastMode();
    void SetBrakeMode();
    
    // Return the value of the sensor connected to the Talon
    int GetEncoderValue();
    void TareEncoder();
    
private:
    
    // Represents information about a single motor in a group
    struct MotorInfo
    {
        TalonSRX * m_pTalonSrx;
        MotorGroupControlMode m_ControlMode;
        int m_CanId;
        
        MotorInfo(MotorGroupControlMode controlMode, int canId) :
            m_pTalonSrx(new TalonSRX(canId)),
            m_ControlMode(controlMode),
            m_CanId(canId)
        {
        }
    };

    static const int MAX_NUMBER_OF_MOTORS = 4;
    static const int FEEDBACK_DEVICE_NONE = 0xFF;           // 2019: CTR removed FeedbackDevice::None with a TODO to restore it

    // Member variables
    int m_NumMotors;                                        // Number of motors in the group
    FeedbackDevice m_Sensor;                                // Keep track of the sensor attached to the Talon (assumes one sensor per group)
    MotorInfo *  m_pMotorsInfo[MAX_NUMBER_OF_MOTORS];       // The motor objects
    
    // Prevent default construction/deletion/copy/assignment
    TalonMotorGroup();
    ~TalonMotorGroup();
    TalonMotorGroup( const TalonMotorGroup& ) = delete;
    TalonMotorGroup & operator=( const TalonMotorGroup& ) = delete;
};

#endif // TALONMOTORGROUP_HPP
