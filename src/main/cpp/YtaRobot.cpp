////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobot.cpp
/// @author David Stalter
///
/// @details
/// Implementation of the YtaRobot class.  This file contains the functions for
/// full robot operation in FRC.  It contains the autonomous and operator
/// control routines as well as all necessary support for interacting with all
/// motors, sensors and input/outputs on the robot.
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <cstddef>                      // for nullptr
#include <cstring>                      // for memset

// C INCLUDES
// (none)

// C++ INCLUDES
#include "YtaRobot.hpp"                 // for class declaration (and other headers)
#include "RobotCamera.hpp"              // for interacting with cameras


////////////////////////////////////////////////////////////////
/// @method YtaRobot::YtaRobot
///
/// Constructor.  Instantiates all robot control objects.
///
////////////////////////////////////////////////////////////////
YtaRobot::YtaRobot() :
    m_AutonomousChooser                 (),
    m_pDriverStation                    (&DriverStation::GetInstance()),
    m_pDriveJoystick                    (nullptr),
    m_pControlJoystick                  (nullptr),
    m_pDriveCustomController            (new YtaController(DRIVE_JOYSTICK_PORT)),
    m_pControlCustomController          (new YtaController(CONTROL_JOYSTICK_PORT)),
    m_pDriveLogitechExtreme             (new Joystick(DRIVE_JOYSTICK_PORT)),
    m_pControlLogitechExtreme           (new Joystick(CONTROL_JOYSTICK_PORT)),
    m_pDriveXboxGameSir                 (new XboxController(DRIVE_JOYSTICK_PORT)),
    m_pControlXboxGameSir               (new XboxController(CONTROL_JOYSTICK_PORT)),
    m_pLeftDriveMotors                  (new TalonMotorGroup(NUMBER_OF_LEFT_DRIVE_MOTORS, LEFT_MOTORS_CAN_START_ID, MotorGroupControlMode::CUSTOM, FeedbackDevice::CTRE_MagEncoder_Relative)),
    m_pRightDriveMotors                 (new TalonMotorGroup(NUMBER_OF_RIGHT_DRIVE_MOTORS, RIGHT_MOTORS_CAN_START_ID, MotorGroupControlMode::CUSTOM, FeedbackDevice::CTRE_MagEncoder_Relative)),
    m_pLedRelay                         (new Relay(LED_RELAY_ID)),
    m_pAutonomousTimer                  (new Timer()),
    m_pInchingDriveTimer                (new Timer()),
    m_pI2cTimer                         (new Timer()),
    m_pCameraRunTimer                   (new Timer()),
    m_pSafetyTimer                      (new Timer()),
    m_pAccelerometer                    (new BuiltInAccelerometer),
    m_pGyro                             (new ADXRS450_Gyro()),
    m_CameraThread                      (RobotCamera::VisionThread),
    m_pToggleFullProcessingTrigger      (new TriggerChangeValues()),
    m_pToggleProcessedImageTrigger      (new TriggerChangeValues()),
    m_SerialPortBuffer                  (),
    m_pSerialPort                       (new SerialPort(SERIAL_PORT_BAUD_RATE, SerialPort::kMXP, SERIAL_PORT_NUM_DATA_BITS, SerialPort::kParity_None, SerialPort::kStopBits_One)),
    m_I2cData                           (),
    m_pI2cPort                          (new I2C(I2C::kMXP, I2C_DEVICE_ADDRESS)),
    m_RobotMode                         (ROBOT_MODE_NOT_SET),
    m_AllianceColor                     (m_pDriverStation->GetAlliance()),
    m_bDriveSwap                        (false),
    m_bLed                              (false)
{
    DisplayMessage("Robot constructor.");
    
    // Set the autonomous options
    m_AutonomousChooser.SetDefaultOption(AUTO_ROUTINE_1_STRING, AUTO_ROUTINE_1_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_2_STRING, AUTO_ROUTINE_2_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_3_STRING, AUTO_ROUTINE_3_STRING);
    m_AutonomousChooser.AddOption(AUTO_TEST_ROUTINE_STRING, AUTO_TEST_ROUTINE_STRING);
    SmartDashboard::PutData("Autonomous Modes", &m_AutonomousChooser);
    
    // Set the driver input to the correct object
    switch (DRIVE_CONTROLLER_TYPE)
    {
        case CUSTOM_CONTROLLER:
        {
            m_pDriveJoystick = m_pDriveCustomController;
            break;
        }
        case LOGITECH_EXTREME:
        {
            m_pDriveJoystick = m_pDriveLogitechExtreme;
            break;
        }
        case LOGITECH_GAMEPAD:
        case XBOX_GAMESIR:
        {
            m_pDriveJoystick = m_pDriveXboxGameSir;
            break;
        }
        default:
        {
            // Deliberately crash - fix the configuration in the header and try again
            ASSERT(false);
            break;
        }
    }
    
    // Set the controller input to the correct object
    switch (CONTROL_CONTROLLER_TYPE)
    {
        case CUSTOM_CONTROLLER:
        {
            m_pControlJoystick = m_pControlCustomController;
            break;
        }
        case LOGITECH_EXTREME:
        {
            m_pControlJoystick = m_pControlLogitechExtreme;
            break;
        }
        case LOGITECH_GAMEPAD:
        case XBOX_GAMESIR:
        {
            m_pControlJoystick = m_pControlXboxGameSir;
            break;
        }
        default:
        {
            // Deliberately crash - fix the configuration in the header and try again
            ASSERT(false);
            break;
        }
    }

    // The drive motors on this robot are custom groups of three.
    // Two drive the same direction, one drives inverse.  Set the parameters.
    ASSERT(m_pLeftDriveMotors->SetMotorInGroupControlMode(LEFT_MOTORS_CAN_START_ID + 1, MotorGroupControlMode::FOLLOW));
    ASSERT(m_pLeftDriveMotors->SetMotorInGroupControlMode(LEFT_MOTORS_CAN_START_ID + 2, MotorGroupControlMode::INVERSE));
    ASSERT(m_pRightDriveMotors->SetMotorInGroupControlMode(RIGHT_MOTORS_CAN_START_ID + 1, MotorGroupControlMode::FOLLOW));
    ASSERT(m_pRightDriveMotors->SetMotorInGroupControlMode(RIGHT_MOTORS_CAN_START_ID + 2, MotorGroupControlMode::INVERSE));
    
    // Reset all timers
    m_pAutonomousTimer->Reset();
    m_pInchingDriveTimer->Reset();
    m_pI2cTimer->Reset();
    m_pCameraRunTimer->Reset();
    m_pSafetyTimer->Reset();

    // Reset the serial port
    m_pSerialPort->Reset();
    
    // Spawn the vision thread
    m_CameraThread.detach();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::RobotInit
///
/// This method is run when initializing the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobot::RobotInit()
{
    DisplayMessage("RobotInit called.");
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::RobotPeriodic
///
/// This method is run in all robot states.  It is called each
/// time a new packet is received from the driver station.
///
////////////////////////////////////////////////////////////////
void YtaRobot::RobotPeriodic()
{
    static bool bRobotPeriodicStarted = false;
    if (!bRobotPeriodicStarted)
    {
        DisplayMessage("RobotPeriodic called.");
        bRobotPeriodicStarted = true;
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::InitialStateSetup
///
/// This method contains the work flow for putting motors,
/// solenoids, etc. into a known state.  It is intended to be
/// used by both autonomous and user control.
///
////////////////////////////////////////////////////////////////
void YtaRobot::InitialStateSetup()
{
    // Start with motors off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    // Configure brake or coast for the drive motors
    m_pLeftDriveMotors->SetBrakeMode();
    m_pRightDriveMotors->SetBrakeMode();
    
    // Tare encoders
    m_pLeftDriveMotors->TareEncoder();
    m_pRightDriveMotors->TareEncoder();
    
    // Stop/clear any timers, just in case
    m_pInchingDriveTimer->Stop();
    m_pInchingDriveTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Start I2C timer
    m_pI2cTimer->Reset();
    m_pI2cTimer->Start();
    
    // Start the camera timer
    m_pCameraRunTimer->Reset();
    m_pCameraRunTimer->Start();
    
    // Just in case constructor was called before these were set (likely the case)
    m_AllianceColor = m_pDriverStation->GetAlliance();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::TeleopInit
///
/// The teleop init method.  This method is called once each
/// time the robot enters teleop control.
///
////////////////////////////////////////////////////////////////
void YtaRobot::TeleopInit()
{
    DisplayMessage("TeleopInit called.");
    
    // Autonomous should have left things in a known state, but
    // just in case clear everything.
    InitialStateSetup();
    
    // Tele-op won't do detailed processing of the images unless instructed to
    RobotCamera::SetFullProcessing(false);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::TeleopPeriodic
///
/// The teleop control method.  This method is called
/// periodically while the robot is in teleop control.
///
////////////////////////////////////////////////////////////////
void YtaRobot::TeleopPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_TELEOP);

    //CheckForDriveSwap();
    DriveControlSequence();
    
    //LedSequence();

    //SolenoidSequence();

    //SonarSensorSequence();
    
    //GyroSequence();

    //SerialPortSequence();
    
    //I2cSequence();
    
    //CameraSequence();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::LedSequence
///
/// This method contains the main workflow for controlling
/// any LEDs on the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobot::LedSequence()
{
    // If the target's in range, give a visual indication
    if (m_bLed)
    {
        m_pLedRelay->Set(Relay::kOn);
    }
    else
    {
        // Otherwise set them off
        m_pLedRelay->Set(Relay::kOff);
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::SolenoidSequence
///
/// This method contains the main workflow for updating the
/// state of the solenoids on the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobot::SolenoidSequence()
{
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::SonarSensorSequence
///
/// This method contains the main workflow for getting updates
/// from the sonar sensors.  In order to not interfere with
/// each other, each sensor is enabled/disabled and checked
/// individually.
///
////////////////////////////////////////////////////////////////
void YtaRobot::SonarSensorSequence()
{
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::GyroSequence
///
/// This method contains the main workflow for getting updates
/// from the gyro sensors and processing related to those
/// readings.
///
////////////////////////////////////////////////////////////////
void YtaRobot::GyroSequence()
{
    GetGyroValue(nullptr);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::SerialPortSequence
///
/// This method contains the main workflow for interaction with
/// the serial port.
///
////////////////////////////////////////////////////////////////
void YtaRobot::SerialPortSequence()
{
    /*
    // Check for any incoming transmissions, limit it to our read buffer size
    int32_t bytesReceived = m_pSerialPort->GetBytesReceived();
    bytesReceived = (bytesReceived > SERIAL_PORT_BUFFER_SIZE_BYTES) ? SERIAL_PORT_BUFFER_SIZE_BYTES : bytesReceived;

    // If we got data, read it
    if (bytesReceived > 0)
    {
        static_cast<void>(m_pSerialPort->Read(m_SerialPortBuffer, bytesReceived));

        // See if its a packet intended for us
        if (memcmp(m_SerialPortBuffer, SERIAL_PORT_PACKET_HEADER, SERIAL_PORT_PACKET_HEADER_SIZE_BYTES) == 0)
        {
            // Next character is the command.  Array indexing starts at zero, thus no +1 on the size bytes constant
            int32_t command = static_cast<int32_t>(m_SerialPortBuffer[SERIAL_PORT_PACKET_HEADER_SIZE_BYTES]) - ASCII_0_OFFSET;

            // Sanity check it
            if (command >= 0 && command <= 9)
            {
                printf("Received a valid packet, command: %d\n", command);
            }
            else
            {
                printf("Invalid command received: %d\n", command);
            }
        }

        printf(m_SerialPortBuffer);
    }
    m_SerialPortBuffer[0] = NULL_CHARACTER;
    */
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::I2cSequence
///
/// This method contains the main workflow for interaction with
/// the I2C bus.
///
////////////////////////////////////////////////////////////////
void YtaRobot::I2cSequence()
{
    //uint8_t I2C_STRING[] = "i2c_roborio";
    std::memset(&m_I2cData, 0U, sizeof(m_I2cData));
    
    if (m_pI2cTimer->Get() > I2C_RUN_INTERVAL_S)
    {
        // Get the data from the riodiuino
        //static_cast<void>(m_pI2cPort->Transaction(I2C_STRING, sizeof(I2C_STRING), reinterpret_cast<uint8_t *>(&m_I2cData), sizeof(m_I2cData)));
        
        // Restart the timer
        m_pI2cTimer->Reset();
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::CameraSequence
///
/// This method handles camera related behavior.  See the
/// RobotCamera class for full details.
///
////////////////////////////////////////////////////////////////
void YtaRobot::CameraSequence()
{
    static bool bFullProcessing = false;
    
    // To not kill the CPU, only do full vision processing (particle analysis) periodically
    if (m_pCameraRunTimer->Get() >= CAMERA_RUN_INTERVAL_S)
    {
        m_pCameraRunTimer->Reset();
    }
    
    // Check for any change in camera
    if (m_pDriveJoystick->GetRawButton(SELECT_FRONT_CAMERA_BUTTON))
    {
        RobotCamera::SetCamera(RobotCamera::FRONT_USB);
    }
    else if (m_pDriveJoystick->GetRawButton(SELECT_BACK_CAMERA_BUTTON))
    {
        RobotCamera::SetCamera(RobotCamera::BACK_USB);
    }
    else
    {
    }
    
    // Look for full processing to be enabled/disabled
    m_pToggleFullProcessingTrigger->m_bCurrentValue = m_pDriveJoystick->GetRawButton(CAMERA_TOGGLE_FULL_PROCESSING_BUTTON);
    if (m_pToggleFullProcessingTrigger->DetectChange())
    {
        // Change state first, because the default is set before this code runs
        bFullProcessing = !bFullProcessing;
        RobotCamera::SetFullProcessing(bFullProcessing);
    }
    
    // Look for the displayed processed image to be changed
    m_pToggleProcessedImageTrigger->m_bCurrentValue = m_pDriveJoystick->GetRawButton(CAMERA_TOGGLE_PROCESSED_IMAGE_BUTTON);
    if (m_pToggleProcessedImageTrigger->DetectChange())
    {
        RobotCamera::ToggleCameraProcessedImage();
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DriveControlSequence
///
/// This method contains the main workflow for drive control.
/// It will gather input from the drive joystick and then filter
/// those values to ensure they are past a certain threshold and
/// within range to send to the speed controllers.  Lastly it
/// will actually set the speed values.
///
////////////////////////////////////////////////////////////////
void YtaRobot::DriveControlSequence()
{
    // Computes what the maximum drive speed could be.
    // It's a little unfortunate we have to handle throttle this way,
    // but GetThrottle is not a member of the GenericHID base class,
    // so we can't use the generic objects since the v-table layout
    // is not the same.  This means we have to manually get the throttle
    // based on the driver input type.
    double throttleControl = 0.0;
    switch (DRIVE_CONTROLLER_TYPE)
    {
        case CUSTOM_CONTROLLER:
        {
            throttleControl = GetThrottleControl(m_pDriveCustomController);
            break;
        }
        case LOGITECH_EXTREME:
        {
            throttleControl = GetThrottleControl(m_pDriveLogitechExtreme);
            break;
        }
        case LOGITECH_GAMEPAD:
        case XBOX_GAMESIR:
        {
            // Xbox controllers have no GetThrottle method, default to max
            throttleControl = 1.0;
            break;
        }
        default:
        {
            // Deliberately crash - fix the configuration in the header and try again
            ASSERT(false);
            break;
        }
    }
    
    // Get driver X/Y inputs
    double xAxisDrive = m_pDriveJoystick->GetX();
    double yAxisDrive = m_pDriveJoystick->GetY();
    
    // Make sure axes inputs clear a certain threshold.  This will help to drive straight.
    xAxisDrive = Trim((xAxisDrive * throttleControl), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    yAxisDrive = Trim((yAxisDrive * throttleControl), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);

    // If the swap direction button was pressed, negate y value
    if ( m_bDriveSwap )
    {
        yAxisDrive *= -1;
    }

    // Filter motor speeds
    double leftSpeed = Limit((xAxisDrive - yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    double rightSpeed = Limit((xAxisDrive + yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    //double leftSpeed = Limit((-xAxisDrive + yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    //double rightSpeed = Limit((-xAxisDrive - yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    
    // Set motor speed
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);
    
    /*
    // First check for inching controls
    if (m_pDriveJoystick->GetRawButton(INCH_FORWARD_BUTTON))
    {
        DirectionalInch(INCHING_DRIVE_SPEED, FORWARD);
    }
    else if (m_pDriveJoystick->GetRawButton(INCH_BACKWARD_BUTTON))
    {
        DirectionalInch(INCHING_DRIVE_SPEED, REVERSE);
    }
    else if (m_pDriveJoystick->GetRawButton(INCH_LEFT_BUTTON))
    {
        DirectionalInch(INCHING_DRIVE_SPEED, LEFT);
    }
    else if (m_pDriveJoystick->GetRawButton(INCH_RIGHT_BUTTON))
    {
        DirectionalInch(INCHING_DRIVE_SPEED, RIGHT);
    }
    else
    {
        // Set motor speed
        m_pLeftDriveMotors->Set(leftSpeed);
        m_pRightDriveMotors->Set(rightSpeed);
    }
    */
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DirectionalInch
///
/// This method contains the main workflow for drive directional
/// inching.  Based on input direction, it will briefly move the
/// robot a slight amount in that direction.
///
////////////////////////////////////////////////////////////////
void YtaRobot::DirectionalInch(double speed, EncoderDirection direction)
{
    // 20xx LEFT FORWARD DRIVE IS NEGATIVE
    // 20xx RIGHT FORWARD DRIVE IS POSITIVE
    double leftSpeed = speed;
    double rightSpeed = speed;
    
    // Negate appropriate motor speeds, based on direction
    switch (direction)
    {
        case FORWARD:
        {
            leftSpeed *= -1.0;
            break;
        }
        case REVERSE:
        {
            rightSpeed *= -1.0;
            break;
        }
        case LEFT:
        {
            break;
        }
        case RIGHT:
        {
            leftSpeed *= -1.0;
            rightSpeed *= -1.0;
            break;
        }
        default:
        {
            break;
        }
    }
    
    // Start the timer
    m_pInchingDriveTimer->Reset();
    m_pInchingDriveTimer->Start();
    
    // Motors on
    m_pLeftDriveMotors->Set(leftSpeed);
    m_pRightDriveMotors->Set(rightSpeed);
    
    while (m_pInchingDriveTimer->Get() < INCHING_DRIVE_DELAY_S)
    {
    }
    
    // Motors back off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    // Stop the timer
    m_pInchingDriveTimer->Stop();
    m_pInchingDriveTimer->Reset();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DisabledInit
///
/// The disabled init method.  This method is called once each
/// time the robot enters disabled mode.
///
////////////////////////////////////////////////////////////////
void YtaRobot::DisabledInit()
{
    DisplayMessage("DisabledInit called.");
    
    // All motors off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::DisabledPeriodic
///
/// The disabled control method.  This method is called
/// periodically while the robot is disabled.
///
////////////////////////////////////////////////////////////////
void YtaRobot::DisabledPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_DISABLED);
}



////////////////////////////////////////////////////////////////
/// @method main
///
/// Execution start for the robt.
///
////////////////////////////////////////////////////////////////
#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<YtaRobot>();
}
#endif
