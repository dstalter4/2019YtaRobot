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
    m_pArmRotationMotors                (new TalonMotorGroup(NUMBER_OF_ARM_ROTATION_MOTORS, ARM_ROTATION_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW)),
    m_pIntakeMotor                      (new TalonSRX(INTAKE_MOTOR_CAN_ID)),
    m_pJackStandMotor                   (new TalonSRX(JACK_STAND_MOTOR_CAN_ID)),
    m_pLedsEnableRelay                  (new Relay(LEDS_ENABLE_RELAY_ID)),
    m_pRedLedRelay                      (new Relay(RED_LED_RELAY_ID)),
    m_pGreenLedRelay                    (new Relay(GREEN_LED_RELAY_ID)),
    m_pBlueLedRelay                     (new Relay(BLUE_LED_RELAY_ID)),
    m_pArmRotationLimitSwitch           (new DigitalInput(ARM_ROTATION_LIMIT_SWITCH_DIO_CHANNEL)),
    m_pLiftBottomLimitSwitch            (new DigitalInput(LIFT_BOTTOM_LIMIT_SWITCH_DIO_CHANNEL)),
    m_pLiftTopLimitSwitch               (new DigitalInput(LIFT_TOP_LIMIT_SWITCH_DIO_CHANNEL)),
    m_pCenterStageLimitSwitch           (new DigitalInput(CENTER_STAGE_LIMIT_SWITCH_DIO_CHANNEL)),
    m_pDebugOutput                      (new DigitalOutput(DEBUG_OUTPUT_DIO_CHANNEL)),
    m_pHatchSolenoid                    (new DoubleSolenoid(HATCH_SOLENOID_FORWARD_CHANNEL, HATCH_SOLENOID_REVERSE_CHANNEL)),
    m_pJackFrontSolenoid                (new DoubleSolenoid(JACK_FRONT_SOLENOID_FORWARD_CHANNEL, JACK_FRONT_SOLENOID_REVERSE_CHANNEL)),
    m_pJackBackSolenoid                 (new DoubleSolenoid(JACK_BACK_SOLENOID_FORWARD_CHANNEL, JACK_BACK_SOLENOID_REVERSE_CHANNEL)),
    m_pJackFrontTrigger                 (nullptr),
    m_pJackBackTrigger                  (nullptr),
    m_pAutonomousTimer                  (new Timer()),
    m_pInchingDriveTimer                (new Timer()),
    m_pDirectionalAlignTimer            (new Timer()),
    m_pSafetyTimer                      (new Timer()),
    m_pAccelerometer                    (new BuiltInAccelerometer),
    m_pAdxrs450Gyro                     (nullptr),
    m_Bno055Angle                       (),
    m_CameraThread                      (RobotCamera::VisionThread),
    m_pToggleFullProcessingTrigger      (nullptr),
    m_pToggleProcessedImageTrigger      (nullptr),
    m_SerialPortBuffer                  (),
    m_pSerialPort                       (new SerialPort(SERIAL_PORT_BAUD_RATE, SerialPort::kMXP, SERIAL_PORT_NUM_DATA_BITS, SerialPort::kParity_None, SerialPort::kStopBits_One)),
    m_I2cThread                         (RobotI2c::I2cThread),
    m_RobotMode                         (ROBOT_MODE_NOT_SET),
    m_RobotDriveState                   (MANUAL_CONTROL),
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
    
    // @todo: Figure out how to assign these sooner to a valid joystick.
    // Since the triggers use a joystick object, they can't be created until the joysticks are assigned
    m_pJackFrontTrigger             = new TriggerChangeValues(m_pDriveJoystick, JACK_FRONT_TOGGLE_BUTTON);
    m_pJackBackTrigger              = new TriggerChangeValues(m_pDriveJoystick, JACK_BACK_TOGGLE_BUTTON);
    m_pToggleFullProcessingTrigger  = new TriggerChangeValues(m_pDriveJoystick, CAMERA_TOGGLE_FULL_PROCESSING_BUTTON);
    m_pToggleProcessedImageTrigger  = new TriggerChangeValues(m_pDriveJoystick, CAMERA_TOGGLE_PROCESSED_IMAGE_BUTTON);
    
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
    
    // Configure the lift, arm and intake motors as brake
    m_pLiftMotors->SetBrakeMode();
    m_pArmRotationMotors->SetBrakeMode();
    m_pIntakeMotor->SetNeutralMode(NeutralMode::Brake);
    
    // Tare encoders
    m_pLeftDriveMotors->TareEncoder();
    m_pRightDriveMotors->TareEncoder();
    
    // Solenoids
    m_pHatchSolenoid->Set(DoubleSolenoid::kOff);
    m_pJackFrontSolenoid->Set(DoubleSolenoid::kOff);
    m_pJackBackSolenoid->Set(DoubleSolenoid::kOff);
    
    // Enable LEDs, but keep them off for now
    m_pLedsEnableRelay->Set(LEDS_ENABLED);
    m_pRedLedRelay->Set(LEDS_OFF);
    m_pGreenLedRelay->Set(LEDS_OFF);
    m_pBlueLedRelay->Set(LEDS_OFF);
    
    // Stop/clear any timers, just in case
    m_pInchingDriveTimer->Stop();
    m_pInchingDriveTimer->Reset();
    m_pDirectionalAlignTimer->Stop();
    m_pDirectionalAlignTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Just in case constructor was called before these were set (likely the case)
    m_AllianceColor = m_pDriverStation->GetAlliance();
    
    // Clear the debug output pin
    m_pDebugOutput->Set(false);
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

    DriveControlSequence();

    LiftAndArmSequence();

    //LedSequence();

    PneumaticSequence();

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
    // Vex and magnetic limit switches read 3.3 when open, 0.0 when closed (touching)
    
    // Lift up/down motors
    // This motor is wired for negative up, positive down
    // Joystick input maps to -1 up, +1 down
    double liftMotorValue = Trim(-m_pControlJoystick->GetRawAxis(MOVE_LIFT_AXIS), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    
    static bool bLiftUpTravelAllowed = true;
    
    // There are extra controls when the lift is moving up
    if (liftMotorValue > 0.0)
    {
        // Check if the limit switches are set
        if ((!m_pCenterStageLimitSwitch->Get()) && (!m_pLiftTopLimitSwitch->Get()))
        {
            // The carriage may overrun its limit switch.
            // If both switches were ever seen set together,
            // indicate that upward motion is not allowed
            // until there is manual input to go back down.
            bLiftUpTravelAllowed = false;
            
            // Upward motion not allowed, downward only
            liftMotorValue = 0.0;
        }
        else if (!bLiftUpTravelAllowed)
        {
            // Execution reaches here if there is still an
            // input to go up, but the limit switch was overshot.
            liftMotorValue = 0.0;
        }
        else
        {
            // Not at the limit, normal motor control
        }
    }
    else
    {
        // Going down or motor off
        bLiftUpTravelAllowed = true;
    }
    
    m_pLiftMotors->Set(-liftMotorValue);
    
    
    
    // Arm rotation motors
    // Joystick input maps to -1 up, +1 down
    double armMotorValue = Trim(-m_pControlJoystick->GetRawAxis(ROTATE_ARM_AXIS), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT) * ARM_ROTATION_MOTOR_SCALING_SPEED;
    
    // Make sure the arm rotation limit switch is not tripped
    if ((armMotorValue > 0.0) && (!m_pArmRotationLimitSwitch->Get()))
    {
        // Upward motion not allowed, downward only
        armMotorValue = 0.0;
    }
    
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
    LedsTest();
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::PneumaticSequence
///
/// This method contains the main workflow for updating the
/// state of the pnemuatics on the robot.
///
////////////////////////////////////////////////////////////////
void YtaRobot::PneumaticSequence()
{
    // Front jack stand control
    static bool bJackFrontRaised = false;
    if (m_pJackFrontTrigger->DetectChange())
    {
        if (!bJackFrontRaised)
        {
            m_pJackFrontSolenoid->Set(DoubleSolenoid::kForward);
        }
        else
        {
            m_pJackFrontSolenoid->Set(DoubleSolenoid::kReverse);
        }

        bJackFrontRaised = !bJackFrontRaised;
    }
    
    // Back jack stand control
    static bool bJackBackRaised = false;
    if (m_pJackBackTrigger->DetectChange())
    {
        if (!bJackBackRaised)
        {
            m_pJackBackSolenoid->Set(DoubleSolenoid::kForward);
        }
        else
        {
            m_pJackBackSolenoid->Set(DoubleSolenoid::kReverse);
        }

        bJackBackRaised = !bJackBackRaised;
    }
    
    // Hatch solenoid control
    if ((m_pDriveJoystick->GetRawButton(DRIVE_HATCH_BUTTON)) || (m_pControlJoystick->GetRawButton(CONTROL_HATCH_BUTTON)))
    {
        m_pHatchSolenoid->Set(DoubleSolenoid::kForward);
    }
    else
    {
        m_pHatchSolenoid->Set(DoubleSolenoid::kReverse);
    }
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
    
    // @note: Use std::chrono is precise time control is needed.
    
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
    if (m_pToggleFullProcessingTrigger->DetectChange())
    {
        // Change state first, because the default is set before this code runs
        bFullProcessing = !bFullProcessing;
        RobotCamera::SetFullProcessing(bFullProcessing);
    }
    
    // Look for the displayed processed image to be changed
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
    // Check for a directional align first
    DirectionalAlign();
    
    // If an align is in progress, do not accept manual driver input
    if (m_RobotDriveState == DIRECTIONAL_ALIGN)
    {
        return;
    }

    //CheckForDriveSwap();
    
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
/// @method YtaRobot::DirectionalAlign
///
/// This method contains the main workflow for automatically
/// aligning the robot to an angle based on input from the
/// driver.  The angles are relative to the robot at the start
/// of the match (when power is applied to the gyro and zero
/// is set).  The robot angle is reported as follows:
///
///     0
///     |
/// 270---90
///     |
///    180
///
/// The POV input is used to pick the angle to align to.  The
/// corresponding input on the d-pad maps 1:1 to the drawing.
///
////////////////////////////////////////////////////////////////
void YtaRobot::DirectionalAlign()
{
    // Retain the last POV value between function invocations
    static int lastPovValue = -1;
    
    // Indicate whether or not a change between align/no align is allowed
    static bool bStateChangeAllowed = false;
    
    // Get the current POV value
    int povValue = m_pDriveJoystick->GetPOV();
    
    // Check if it changed since last function call
    if (povValue != lastPovValue)
    {
        // Something changed, figure out what
        
        // POV button was released
        if (povValue == -1)
        {
            // State change not allowed until next button press
            bStateChangeAllowed = false;
        }
        // POV button was pressed
        else if (lastPovValue == -1)
        {
            // State change allowed since button is now pressed
            bStateChangeAllowed = true;
        }
        // There was some change in the already pressed POV value, which doesn't matter
        else
        {
        }
    }
    
    const int POV_NORMALIZATION_ANGLE = 45;
    
    // Save off a new last POV value
    lastPovValue = povValue;
    
    // This alignment uses the following from the POV input:
    //
    // ///////////////////////
    // //   315      45     //
    // //     \  up  /      //
    // // left |    | right //
    // //     / down \      //
    // //   225      135    //
    // ///////////////////////
    //
    // The input value (0 -> 360) will be normalized such that
    // angle 315 is interpreted as zero.
    static int destinationAngle = -1;
    
    switch (m_RobotDriveState)
    {
        case MANUAL_CONTROL:
        {
            // Only start an align if a state change is allowed
            if (bStateChangeAllowed)
            {                
                // This gives a value between 45 -> 405
                povValue += POV_NORMALIZATION_ANGLE;
                
                // Normalize between 0 -> 360 (maps 0:360 in to 45:360:0:45 out)
                if (povValue >= ANGLE_360_DEGREES)
                {
                    povValue -= ANGLE_360_DEGREES;
                }
                
                // Now at value between 0 -> 360, where:
                // 0 -> 89 = align up
                // 90 -> 179 = align right
                // 180 -> 269 = align down
                // 270 -> 359 = align left
                // Get a scalar multiplier to find the destination angle.
                // Making this volatile to prevent the compiler from trying
                // to optimize the division followed by multliplication of
                // the same constant.  Integer division is deliberate.
                // This gives a scalar multiplier of 0 -> 3
                volatile int degreeMultiplier = (povValue / ANGLE_90_DEGREES);
                
                // Find the destination angle.
                // This gives a value of 0, 90, 180 or 270
                destinationAngle = ANGLE_90_DEGREES * degreeMultiplier;
                
                // Read the starting angle
                RobotI2c::ManualTrigger();
                int startingAngle = static_cast<int>(GetGyroValue(BNO055));
                
                // Do some angle math to figure out which direction is faster to turn.
                // Examples:
                // Starting: 45, 180    Destination: 0, 90, 180, 270
                // 45 - 0 = 45          180 - 0 = 180
                // 45 - 90 = -45        180 - 90 = 90
                // 45 - 180 = -135      180 - 180 = 0
                // 45 - 270 = -225      180 - 270 = -90
                int angleDistance = startingAngle - destinationAngle;
                int absValueAngleDistance = std::abs(angleDistance);
                
                // Variables to indicate which way to turn
                bool bTurnLeft = false;
                bool bTurnRight = false;
                
                // Figure out which way to turn
                if (angleDistance > 0)
                {
                    // Target is to the left of where we are
                    bTurnLeft = true;
                }
                else
                {
                    // Target is to the right of where we are
                    bTurnRight = true;
                }

                // If the target distance is more than halfway around, it's actually faster to turn the other way 
                if (absValueAngleDistance > ANGLE_180_DEGREES)
                {
                    bTurnLeft = !bTurnLeft;
                    bTurnRight = !bTurnRight;
                }
                
                // The destination angle and direction is now known, time to do the move
                if (bTurnLeft)
                {
                    m_pLeftDriveMotors->Set(-DIRECTIONAL_ALIGN_DRIVE_SPEED);
                    m_pRightDriveMotors->Set(-DIRECTIONAL_ALIGN_DRIVE_SPEED);
                }
                if (bTurnRight)
                {
                    m_pLeftDriveMotors->Set(DIRECTIONAL_ALIGN_DRIVE_SPEED);
                    m_pRightDriveMotors->Set(DIRECTIONAL_ALIGN_DRIVE_SPEED);
                }
                
                // Start the safety timer
                m_pDirectionalAlignTimer->Start();

                // Indicate a state change is not allowed until POV release
                bStateChangeAllowed = false;
                
                // Indicate a directional align is in process
                m_RobotDriveState = DIRECTIONAL_ALIGN;
            }
            
            break;
        }
        case DIRECTIONAL_ALIGN:
        {   
            // Force update gyro value
            RobotI2c::ManualTrigger();
            
            // Three conditions for stopping the align:
            // 1. Destination angle is reached
            // 2. Safety timer expires
            // 3. User cancels the operation
            // @todo: Is it a problem that (destinationAngle - 1) can be negative when angle == zero?
            int currentAngle = static_cast<int>(GetGyroValue(BNO055));
            if (((currentAngle >= (destinationAngle - 1)) && (currentAngle <= (destinationAngle + 1))) ||
                (m_pDirectionalAlignTimer->Get() > DIRECTIONAL_ALIGN_MAX_TIME_S) ||
                (bStateChangeAllowed))
            {
                // Motors off
                m_pLeftDriveMotors->Set(OFF);
                m_pRightDriveMotors->Set(OFF);
                
                // Reset the safety timer
                m_pDirectionalAlignTimer->Stop();
                m_pDirectionalAlignTimer->Reset();
                
                // Clear this just to be safe
                destinationAngle = -1;
                
                // Indicate a state change is not allowed until POV release
                bStateChangeAllowed = false;
                
                // Align done, back to manual control
                m_RobotDriveState = MANUAL_CONTROL;
            }
            
            break;
        }
        default:
        {
            break;
        }
    }
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
