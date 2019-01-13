////////////////////////////////////////////////////////////////////////////////
/// @file   TalonMotorGroup.cpp
/// @author David Stalter
///
/// @details
/// A class designed to work with a group of CAN Talon speed controllers working
/// in tandem.
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "TalonMotorGroup.hpp"      // for class declaration

// STATIC MEMBER DATA
// (none)



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::TalonMotorGroup
///
/// Constructor.  Creates the number of motors specified on the
/// port numbers passed in.
///
////////////////////////////////////////////////////////////////
TalonMotorGroup::TalonMotorGroup( int numMotors, int firstCANId, MotorGroupControlMode controlMode, FeedbackDevice sensor )
: m_NumMotors(numMotors)
, m_pMotors()
, m_ControlMode(controlMode)
, m_Sensor(sensor)
{
    // Allocate the necessary storage for all of the objects by invoking operator new[]
    // Assign the returned memory block to the first pointer in the array
    //m_pMotors[0] =  reinterpret_cast<TalonSRX *>( operator new[] (numMotors * sizeof(TalonSRX)) );

    // CAN Talons can be set to follow, which the motor groups
    // may do, so save off the first id as the master
    int masterId = firstCANId;

    // Loop for each motor to create
    for ( int i = 0; i < numMotors; i++ )
    {
        // Create it
        m_pMotors[i] = new TalonSRX(firstCANId++);
        
        // Override to always coast
        m_pMotors[i]->SetNeutralMode(NeutralMode::Coast);

        // Only set follow for Talon groups that will be configured
        // as such.  Otherwise just use the defaults (percent voltage based).
        // The CTRE Phoenix library now passes the control mode in the
        // Set() method, so we only need to set the followers here.
        if ((i != 0) && (controlMode == FOLLOW))
        {
            m_pMotors[i]->Set(ControlMode::Follower, masterId);
        }
        else if (i == 0)
        {
            // This assumes only the first controller in a group has a sensor.
            
            // Sensor initialization (feedbackDevice, pidIdx, timeoutMs)
            m_pMotors[i]->ConfigSelectedFeedbackSensor(sensor, 0, 0);
        }
        else
        {
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetCoastMode
///
/// Method to change a talon to coast mode.
///
////////////////////////////////////////////////////////////////
void TalonMotorGroup::SetCoastMode()
{
    for (int i = 0; i < m_NumMotors; i++)
    {
        m_pMotors[i]->SetNeutralMode(NeutralMode::Coast);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetBrakeMode
///
/// Method to change a talon to brake mode.
///
////////////////////////////////////////////////////////////////
void TalonMotorGroup::SetBrakeMode()
{
    for (int i = 0; i < m_NumMotors; i++)
    {
        m_pMotors[i]->SetNeutralMode(NeutralMode::Brake);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::TareEncoder
///
/// Method to tare the value on an encoder feedback device
/// connected to a Talon controller.
///
////////////////////////////////////////////////////////////////
void TalonMotorGroup::TareEncoder()
{
    if (m_Sensor == FeedbackDevice::CTRE_MagEncoder_Relative)
    {
        // sensorPos, pidIdx, timeoutMs
        m_pMotors[0]->SetSelectedSensorPosition(0, 0, 0);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::GetEncoderValue
///
/// Method to get the value from an encoder feedback device
/// connected to a Talon controller.
///
////////////////////////////////////////////////////////////////
int TalonMotorGroup::GetEncoderValue()
{
    if (m_Sensor == FeedbackDevice::CTRE_MagEncoder_Relative)
    {
        // pidIdx
        return m_pMotors[0]->GetSelectedSensorPosition(0);
    }
    else
    {
        return 0;
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::Set
///
/// Method to set the speed of each motor in the group.
///
////////////////////////////////////////////////////////////////
void TalonMotorGroup::Set( double value )
{
    // Check what kind of group this is.  Most
    // CAN Talons will be set to follow, but some
    // may be independent or inverse (such as if
    // they need to drive in different directions).
    switch (m_ControlMode)
    {
        // Typical case, just update the master
        case FOLLOW:
        {
            m_pMotors[0]->Set(ControlMode::PercentOutput, value);
            break;
        }
        case INDEPENDENT:
        {
            for (int i = 0; i < m_NumMotors; i++)
            {
                m_pMotors[i]->Set(ControlMode::PercentOutput, value);
            }
            break;
        }
        // Motors are attached to drive in
        // opposite directions
        case INVERSE:
        {
            // Assumes each half of motors need to go the same direction
            // (i.e. 1:n motors, 1:n/2 forward, n/2:n reverse
            for (int i = 0; i < m_NumMotors / 2; i++)
            {
               m_pMotors[i]->Set(ControlMode::PercentOutput, value);
            }
            for (int i = m_NumMotors / 2; i < m_NumMotors; i++)
            {
                   m_pMotors[i]->Set(ControlMode::PercentOutput, -value);
            }
            break;
        }
        // Default cases for the offsets
        case INDEPENDENT_OFFSET:
        case INVERSE_OFFSET:
        {
            SetWithOffset(value, value);
            break;
        }
        default:
        {
            break;
        }
    };
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetWithOffset
///
/// Method to set the speed of each motor in the group, where
/// the speed is different between motors in the group.
///
////////////////////////////////////////////////////////////////
void TalonMotorGroup::SetWithOffset( double group1Value, double group2Value )
{
    // Check what kind of group this is.  This Talon
    // group is not uniform, so different values need
    // to be applied.
    switch (m_ControlMode)
    {
        case INDEPENDENT_OFFSET:
        {
            // Assumes each half of motors need to go the same direction
            // (i.e. 1:n motors, 1:n/2 forward, n/2:n reverse
            for (int i = 0; i < m_NumMotors / 2; i++)
            {
                m_pMotors[i]->Set(ControlMode::PercentOutput, group1Value);
            }
            for (int i = m_NumMotors / 2; i < m_NumMotors; i++)
            {
                m_pMotors[i]->Set(ControlMode::PercentOutput, group2Value);
            }
            break;
        }
        // Motors are attached to drive in
        // opposite directions
        case INVERSE_OFFSET:
        {
            // Assumes each half of motors need to go the same direction
            // (i.e. 1:n motors, 1:n/2 forward, n/2:n reverse
            for (int i = 0; i < m_NumMotors / 2; i++)
            {
                m_pMotors[i]->Set(ControlMode::PercentOutput, group1Value);
            }
            for (int i = m_NumMotors / 2; i < m_NumMotors; i++)
            {
                m_pMotors[i]->Set(ControlMode::PercentOutput, -group2Value);
            }
            break;
        }
        default:
        {
            break;
        }
    };
}
