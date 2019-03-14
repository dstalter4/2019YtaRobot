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
#include "RobotI2c.hpp"                 // for I2cThread()
#include "RobotUtils.hpp"               // for DisplayMessage()

// STATIC MEMBER VARIABLES
YtaRobot * YtaRobot::m_pThis;


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
    m_pLeftDriveMotors                  (new TalonMotorGroup(NUMBER_OF_LEFT_DRIVE_MOTORS, LEFT_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW, FeedbackDevice::CTRE_MagEncoder_Relative)),
    m_pRightDriveMotors                 (new TalonMotorGroup(NUMBER_OF_RIGHT_DRIVE_MOTORS, RIGHT_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW, FeedbackDevice::CTRE_MagEncoder_Relative)),
    m_pLiftMotors                       (new TalonMotorGroup(NUMBER_OF_LIFT_MOTORS, LIFT_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW)),
    m_pArmRotationMotors                (new TalonMotorGroup(NUMBER_OF_ARM_ROTATION_MOTORS, ARM_ROTATION_MOTORS_CAN_START_ID, MotorGroupControlMode::INVERSE)),
    m_pIntakeMotor                      (new TalonSRX(INTAKE_MOTOR_CAN_ID)),
    m_pJackStandMotor                   (new TalonSRX(JACK_STAND_MOTOR_CAN_ID)),
    m_pLedsEnableRelay                  (new Relay(LEDS_ENABLE_RELAY_ID)),
    m_pRedLedRelay                      (new Relay(RED_LED_RELAY_ID)),
    m_pGreenLedRelay                    (new Relay(GREEN_LED_RELAY_ID)),
    m_pBlueLedRelay                     (new Relay(BLUE_LED_RELAY_ID)),
    m_pHatchSolenoid                    (new DoubleSolenoid(HATCH_SOLENOID_FORWARD_CHANNEL, HATCH_SOLENOID_REVERSE_CHANNEL)),
    m_pJackStandSolenoid                (new DoubleSolenoid(JACK_STAND_SOLENOID_FORWARD_CHANNEL, JACK_STAND_SOLENOID_REVERSE_CHANNEL)),
    m_pAutonomousTimer                  (new Timer()),
    m_pInchingDriveTimer                (new Timer()),
    m_pCameraRunTimer                   (new Timer()),
    m_pSafetyTimer                      (new Timer()),
    m_pAccelerometer                    (new BuiltInAccelerometer),
    m_pAdxrs450Gyro                     (nullptr),
    m_Bno055Angle                       (),
    m_CameraThread                      (RobotCamera::VisionThread),
    m_pToggleFullProcessingTrigger      (new TriggerChangeValues()),
    m_pToggleProcessedImageTrigger      (new TriggerChangeValues()),
    m_SerialPortBuffer                  (),
    m_pSerialPort                       (new SerialPort(SERIAL_PORT_BAUD_RATE, SerialPort::kMXP, SERIAL_PORT_NUM_DATA_BITS, SerialPort::kParity_None, SerialPort::kStopBits_One)),
    m_I2cThread                         (RobotI2c::I2cThread),
    m_RobotMode                         (ROBOT_MODE_NOT_SET),
    m_AllianceColor                     (m_pDriverStation->GetAlliance()),
    m_bDriveSwap                        (false)
{
    RobotUtils::DisplayMessage("Robot constructor.");
    
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
    
    // Construct the ADXRS450 gyro if configured
    if (ADXRS450_GYRO_PRESENT)
    {
        m_pAdxrs450Gyro = new ADXRS450_Gyro();
    }

    // Reset the serial port and clear buffer
    m_pSerialPort->Reset();
    std::memset(&m_SerialPortBuffer, 0U, sizeof(m_SerialPortBuffer));
    
    // Spawn the vision and I2C threads
    m_CameraThread.detach();
    m_I2cThread.detach();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::RobotInit
///
/// This method is run when initializing the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobot::RobotInit()
{
    RobotUtils::DisplayMessage("RobotInit called.");
    SetStaticThisInstance();
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
        RobotUtils::DisplayMessage("RobotPeriodic called.");
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
    m_pLiftMotors->Set(OFF);
    m_pArmRotationMotors->Set(OFF);
    m_pIntakeMotor->Set(ControlMode::PercentOutput, OFF);
    m_pJackStandMotor->Set(ControlMode::PercentOutput, OFF);
    
    // Configure brake or coast for the drive motors
    m_pLeftDriveMotors->SetBrakeMode();
    m_pRightDriveMotors->SetBrakeMode();
    
    // Configure lift and arm motors as brake
    m_pLiftMotors->SetBrakeMode();
    m_pArmRotationMotors->SetBrakeMode();
    
    // Tare encoders
    m_pLeftDriveMotors->TareEncoder();
    m_pRightDriveMotors->TareEncoder();
    
    // Solenoids
    m_pHatchSolenoid->Set(DoubleSolenoid::kOff);
    m_pJackStandSolenoid->Set(DoubleSolenoid::kOff);
    
    // Enable LEDs, but keep them off for now
    m_pLedsEnableRelay->Set(LEDS_ENABLED);
    m_pRedLedRelay->Set(LEDS_OFF);
    m_pGreenLedRelay->Set(LEDS_OFF);
    m_pBlueLedRelay->Set(LEDS_OFF);
    
    // Stop/clear any timers, just in case
    m_pInchingDriveTimer->Stop();
    m_pInchingDriveTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
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
    RobotUtils::DisplayMessage("TeleopInit called.");
    
    // Autonomous should have left things in a known state, but
    // just in case clear everything.
    InitialStateSetup();
    
    // Tele-op won't do detailed processing of the images unless instructed to
    RobotCamera::SetFullProcessing(false);
    
    // Indicate to the I2C thread to get data less often
    RobotI2c::SetThreadUpdateRate(I2C_RUN_INTERVAL_MS);
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

    LiftAndArmSequence();

    //LedSequence();

    //SolenoidSequence();

    //SerialPortSequence();
    
    //I2cSequence();
    
    //CameraSequence();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::LiftAndArmSequence
///
/// This method contains the main workflow for controlling
/// the lift and arm on the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobot::LiftAndArmSequence()
{
    // Lift up/down motors
    // Joystick input maps to -1 up, +1 down
    double liftMotorValue = Trim(-m_pControlJoystick->GetRawAxis(MOVE_LIFT_AXIS), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    m_pLiftMotors->Set(-liftMotorValue);
    
    // Arm rotation motors
    // Joystick input maps to -1 up, +1 down
    double armMotorValue = Trim(-m_pControlJoystick->GetRawAxis(ROTATE_ARM_AXIS), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT) * ARM_ROTATION_MOTOR_SCALING_SPEED;
    m_pArmRotationMotors->Set(armMotorValue);
    
    // Intake motor
    if (m_pControlJoystick->GetRawButton(INTAKE_SPIN_IN_BUTTON))
    {
        m_pIntakeMotor->Set(ControlMode::PercentOutput, INTAKE_MOTOR_SPEED);
    }
    // Control input just needs to be non-zero (not using proportional motor control on the intake)
    else if (Trim(m_pControlJoystick->GetRawAxis(INTAKE_SPIN_OUT_AXIS), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT) != 0.0)
    {
        m_pIntakeMotor->Set(ControlMode::PercentOutput, -INTAKE_MOTOR_SPEED);
    }
    else
    {
        m_pIntakeMotor->Set(ControlMode::PercentOutput, OFF);
    }
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
                RobotUtils::DisplayFormattedMessage("Received a valid packet, command: %d\n", command);
            }
            else
            {
                RobotUtils::DisplayFormattedMessage("Invalid command received: %d\n", command);
            }
        }

        RobotUtils::DisplayFormattedMessage(m_SerialPortBuffer);
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
    /*
    static std::chrono::time_point<std::chrono::high_resolution_clock> currentTime;
    static std::chrono::time_point<std::chrono::high_resolution_clock> oldTime;
    currentTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = currentTime - oldTime;
    if (elapsed.count() > 1000)
    {
        double angle = GetGyroValue(BNO055);
        RobotUtils::DisplayFormattedMessage("BNO055: %f\n", angle);
        oldTime = currentTime;
    }
    */
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
    
    // Get the slow drive control joystick input
    double xAxisSlowDrive = m_pDriveJoystick->GetRawAxis(DRIVE_SLOW_X_AXIS);
    xAxisSlowDrive = Trim((xAxisSlowDrive * DRIVE_SLOW_THROTTLE_VALUE), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    
    // If the normal x-axis drive is non-zero, use it.  Otherwise use the slow drive input, which could also be zero.
    xAxisDrive = (xAxisDrive != 0.0) ? xAxisDrive : xAxisSlowDrive;
    
    // Filter motor speeds
    double leftSpeed = Limit((xAxisDrive - yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    double rightSpeed = Limit((xAxisDrive + yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    
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
    RobotUtils::DisplayMessage("DisabledInit called.");
    
    // All motors off
    m_pLeftDriveMotors->Set(OFF);
    m_pRightDriveMotors->Set(OFF);
    
    // Even though 'Disable' shuts off the relay signals, explitily turn the LEDs off
    m_pLedsEnableRelay->Set(LEDS_DISABLED);
    m_pRedLedRelay->Set(LEDS_OFF);
    m_pGreenLedRelay->Set(LEDS_OFF);
    m_pBlueLedRelay->Set(LEDS_OFF);
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
