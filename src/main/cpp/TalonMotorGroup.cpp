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
/// Constructor.  Creates the number of motors specified
/// starting from the CAN ID passed in.
///
////////////////////////////////////////////////////////////////
TalonMotorGroup::TalonMotorGroup( int numMotors, int masterCanId, MotorGroupControlMode nonMasterControlMode, FeedbackDevice sensor ) :
    m_NumMotors(numMotors),
    m_Sensor(sensor)
{
    // Loop for each motor to create
    for ( int i = 0; (i < numMotors) && (i < MAX_NUMBER_OF_MOTORS); i++ )
    {
        // The master Talon is unique
        if (i == 0)
        {
            // Create it
            m_pMotorsInfo[i] = new MotorInfo(MASTER, masterCanId);
            
            // This assumes only the first controller in a group has a sensor
            if (sensor != static_cast<FeedbackDevice>(FEEDBACK_DEVICE_NONE))
            {
                // Sensor initialization (feedbackDevice, pidIdx, timeoutMs)
                m_pMotorsInfo[0]->m_pTalonSrx->ConfigSelectedFeedbackSensor(sensor, 0, 0);
            }
        }
        // Non-master Talons
        else
        {
            // Create it
            m_pMotorsInfo[i] = new MotorInfo(nonMasterControlMode, (masterCanId + i));

            // Only set follow for Talon groups that will be configured as
            // such.  The CTRE Phoenix library now passes the control mode in
            // the Set() method, so we only need to set the followers here.
            if (nonMasterControlMode == FOLLOW)
            {
                m_pMotorsInfo[i]->m_pTalonSrx->Set(ControlMode::Follower, masterCanId);
            }
        }
        
        // Override to always coast
        m_pMotorsInfo[i]->m_pTalonSrx->SetNeutralMode(NeutralMode::Coast);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::AddMotorToGroup
///
/// Method to add a new motor to a motor group.
///
////////////////////////////////////////////////////////////////
bool TalonMotorGroup::AddMotorToGroup(MotorGroupControlMode controlMode)
{
    bool bResult = false;

    // Make sure there's room for another motor in this group
    if (m_NumMotors < MAX_NUMBER_OF_MOTORS)
    {
        // The new motor CAN ID is the first motor's ID + current number of group motors present
        int newMotorCanId = m_pMotorsInfo[0]->m_CanId + m_NumMotors;

        // m_NumMotors can be leveraged as the index, as it represents the next unused array element
        m_pMotorsInfo[m_NumMotors] = new MotorInfo(controlMode, newMotorCanId);

        // Increase the number of motors
        m_NumMotors++;
        
        // Indicate success
        bResult = true;
    }

    return bResult;
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetMotorInGroupControlMode
///
/// Method to set the control mode of a motor in a group.
///
////////////////////////////////////////////////////////////////
bool TalonMotorGroup::SetMotorInGroupControlMode(int canId, MotorGroupControlMode controlMode)
{
    bool bResult = false;
    
    // Search for the correct motor in the group
    for (int i = 0; i < m_NumMotors; i++)
    {
        // If it matches...
        if (m_pMotorsInfo[i]->m_CanId == canId)
        {
            // ...set the control mode
            m_pMotorsInfo[i]->m_ControlMode = controlMode;
            
            // Indicate success
            bResult = true;
        }
    }

    return bResult;
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
        m_pMotorsInfo[i]->m_pTalonSrx->SetNeutralMode(NeutralMode::Coast);
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
        m_pMotorsInfo[i]->m_pTalonSrx->SetNeutralMode(NeutralMode::Brake);
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
        m_pMotorsInfo[0]->m_pTalonSrx->SetSelectedSensorPosition(0, 0, 0);
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
    int sensorValue = 0;

    if (m_Sensor == FeedbackDevice::CTRE_MagEncoder_Relative)
    {
        // pidIdx
        sensorValue = m_pMotorsInfo[0]->m_pTalonSrx->GetSelectedSensorPosition(0);
    }
    
    return sensorValue;
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::Set
///
/// Method to set the speed of each motor in the group.  The
/// offset parameter is only valid for motor groups configured
/// as *_OFFSET.
///
////////////////////////////////////////////////////////////////
void TalonMotorGroup::Set( double value, double offset )
{
    for (int i = 0; i < m_NumMotors; i++)
    {
        // Setting motor values for groups assumes that the first half of
        // motors in a group should always get the same value, and the second
        // half of motors in a group could be different (such as inverse or offset).
        // Keep track of which segment of the motor group this motor is in.
        
        // Enable this control if groups with > 2 motors are needed.
        //bool bInFirstHalf = (i < (m_NumMotors / 2));
        
        // Most modes wil need to call Set() later, but some won't
        bool bCallSet = true;
        
        // The value that will be passed to Set()
        double valueToSet = 0.0;
        
        // Check what the control mode of this motor is.  Most CAN Talons
        // will be set to follow, but some may be independent or inverse (such
        // as if they need to drive in different directions).
        switch (m_pMotorsInfo[i]->m_ControlMode)
        {
            case MASTER:
            case INDEPENDENT:
            {
                // The master always gets set via percent voltage, as do
                // motors that are independently controlled (not follow or inverse).
                valueToSet = value;
                break;
            }
            case FOLLOW:
            {
                // Nothing to do, motor had Set() called during object construction
                bCallSet = false;
                break;
            }
            case INVERSE:
            {
                // Motor is attached to drive in opposite direction of master
                valueToSet = -value;
                break;
            }
            case INDEPENDENT_OFFSET:
            {
                // The non-master motor has a different value in this case
                valueToSet = value + offset;
                break;
            }
            case INVERSE_OFFSET:
            {
                // The non-master motor has a different value in this case
                valueToSet = -(value + offset);
                break;
            }
            default:
            {
                // Can reach here with CUSTOM motors still set.  Calling code should
                // update those motors to a different control mode via class API calls.
                break;
            }
            
            if (bCallSet)
            {
                // Set the value in the Talon
                m_pMotorsInfo[i]->m_pTalonSrx->Set(ControlMode::PercentOutput, valueToSet);
            }
        };
    }
}
