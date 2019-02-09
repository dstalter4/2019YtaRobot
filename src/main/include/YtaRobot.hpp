////////////////////////////////////////////////////////////////////////////////
/// @file   YtaRobot.hpp
/// @author David Stalter
///
/// @details
/// This is the class declaration for a FRC robot derived from the WPI library
/// base classes.  The TimedRobot class is the base of a robot application that
/// will automatically call appropriate Autonomous and Teleop methods at the
/// right time as controlled by the switches on the driver station or the field
/// controls.
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef YTAROBOT_HPP
#define YTAROBOT_HPP

// SYSTEM INCLUDES
#include <cmath>                                // for M_PI

// C INCLUDES
#include "frc/WPILib.h"                         // for FRC library

// C++ INCLUDES
#include "RobotI2c.hpp"                         // for GetGyroData()
#include "RobotUtils.hpp"                       // for ASSERT, DEBUG_PRINTS
#include "TalonMotorGroup.hpp"                  // for Talon group motor control
#include "YtaController.hpp"                    // for custom controller interaction

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class YtaRobot
///
/// Derived class from TimedRobot.  The object that will
/// control all robot functionality.
///
////////////////////////////////////////////////////////////////
class YtaRobot : public TimedRobot
{
public:

    // MEMBER FUNCTIONS
    
    // Base robot routines
    virtual void RobotInit() override;
    virtual void RobotPeriodic() override;
    
    // Autonomous routines
    virtual void AutonomousInit() override;
    virtual void AutonomousPeriodic() override;
    
    // Teleop routines
    virtual void TeleopInit() override;
    virtual void TeleopPeriodic() override;
    
    // Test mode routines
    virtual void TestInit() override;
    virtual void TestPeriodic() override;
    
    // Robot disabled routines
    virtual void DisabledInit() override;
    virtual void DisabledPeriodic() override;
    
    // Constructor, destructor, copy, assignment
    YtaRobot();
    virtual ~YtaRobot() = default;
    YtaRobot(YtaRobot&& rhs) = default;
    YtaRobot& operator=(YtaRobot&& rhs) = default;
      
private:

    // TYPEDEFS
    typedef DriverStation::Alliance Alliance;
    typedef GenericHID::JoystickHand JoystickHand;
    typedef TalonMotorGroup::MotorGroupControlMode MotorGroupControlMode;
    
    // ENUMS
    enum RobotMode
    {
        ROBOT_MODE_AUTONOMOUS,
        ROBOT_MODE_TELEOP,
        ROBOT_MODE_TEST,
        ROBOT_MODE_DISABLED,
        ROBOT_MODE_NOT_SET
    };

    enum ControllerType
    {
        CUSTOM_CONTROLLER,
        LOGITECH_EXTREME,
        LOGITECH_GAMEPAD,
        XBOX_GAMESIR
    };
    
    enum EncoderDirection
    {
        FORWARD,
        REVERSE,
        LEFT,
        RIGHT
    };
    
    enum GyroDirection
    {
        LEFT_TURN,
        RIGHT_TURN
    };

    enum GyroType
    {
        ADXRS450,
        ANALOG,
        BNO055
    };
    
    enum SonarDriveState
    {
        NONE            = 0x00,
        LEFT_GUIDE      = 0x01,
        RIGHT_GUIDE     = 0x02,
        FORWARD_GUIDE   = 0x10,
        REVERSE_GUIDE   = 0x20
    };
    
    enum SonarDriveDirection
    {
        STOPPED,
        SONAR_FORWARD,
        SONAR_REVERSE
    };
    
    // STRUCTS
    struct TriggerChangeValues
    {
        // Detect that a button has been pressed (rather than released)
        inline bool DetectChange();
        
        bool m_bCurrentValue;
        bool m_bOldValue;
    };
    
    // Checks for a robot state change and logs a message if so
    inline void CheckAndUpdateRobotMode(RobotMode robotMode);
    
    // Ensures a number is between the upper and lower bounds
    inline double Limit( double num, double upper, double lower );
    
    // Trims a number to be in between the upper and lower bounds
    inline double Trim( double num, double upper, double lower );

    // Function to check for drive control direction swap
    inline void CheckForDriveSwap();
    
    // Get a throttle control value from a joystick
    inline double GetThrottleControl(Joystick * pJoystick);
    inline double GetThrottleControl(YtaController * pController);

    // Grabs a value from a sonar sensor individually
    inline double GetSonarSensorValue(Ultrasonic * pSensor);
   
    // Get a reading from the gyro sensor
    inline double GetGyroValue(GyroType gyroType, AnalogGyro * pSensor = nullptr);
    
    // Convert a distance in inches to encoder turns
    int GetEncoderRotationsFromInches(int inches, double diameter, bool bUseQuadEncoding = true);

    // Autonomous wait for something to complete delay routine
    inline void AutonomousDelay(double time);

    // Autonomous drive for a specified time
    inline void AutonomousDriveSequence(double speed, double time);
    
    // Autonomous routines to back drive the motors to abruptly stop
    inline void AutonomousBackDrive(EncoderDirection currentRobotDirection);
    inline void AutonomousBackDriveTurn(GyroDirection currentGyroDirection);
    
    // Autonomous routines
    void AutonomousRoutine1();
    void AutonomousRoutine2();
    void AutonomousRoutine3();
    void AutonomousCommon();
    void AutonomousCommonRed();
    void AutonomousCommonBlue();
    bool AutonomousGyroLeftTurn(double destAngle, double turnSpeed);
    bool AutonomousGyroRightTurn(double destAngle, double turnSpeed);
    void AutonomousEncoderDrive(double speed, double distance, EncoderDirection direction);
    bool AutonomousSonarDrive(SonarDriveDirection driveDirection, SonarDriveState driveState, uint32_t destLateralDist, uint32_t destSideDist);

    // Routine to put things in a known state
    void InitialStateSetup();

    // Main sequence for drive motor control
    void DriveControlSequence();
    void SideDriveSequence();
    
    // Function to automate slightly moving the robot
    void DirectionalInch(double speed, EncoderDirection direction);

    // Main sequence for LED control
    void LedSequence();

    // Main sequence for updating solenoid states
    void SolenoidSequence();

    // Main sequence for interaction with the serial port
    void SerialPortSequence();
    
    // Main sequence for I2C interaction
    void I2cSequence();
    
    // Main sequence for vision processing
    void CameraSequence();
    
    // Test routines for trying out experimental code
    void AutonomousTestCode();
    void TeleopTestCode();
    void MotorTest();
    void TankDrive();
    
    // MEMBER VARIABLES
    
    // Autonomous
    SendableChooser<std::string>    m_AutonomousChooser;                    // Selects from the dashboard which auto routine to run
    
    // User Controls
    DriverStation *                 m_pDriverStation;                       // Driver station object for getting selections
    GenericHID *                    m_pDriveJoystick;                       // Base class object for the driver operator
    GenericHID *                    m_pControlJoystick;                     // Base class object for the controller operator
    YtaController *                 m_pDriveCustomController;               // Option 1: Custom interface
    YtaController *                 m_pControlCustomController;             // Option 1: Custom interface
    Joystick *                      m_pDriveLogitechExtreme;                // Option 2: Logitech Extreme can use the Joystick class
    Joystick *                      m_pControlLogitechExtreme;              // Option 2: Logitech Extreme can use the Joystick class
    XboxController *                m_pDriveXboxGameSir;                    // Option 3: Xbox-based controller (also works for Logitech Gamepad)
    XboxController *                m_pControlXboxGameSir;                  // Option 3: Xbox-based controller (also works for Logitech Gamepad)
    
    // Motors
    TalonMotorGroup *               m_pLeftDriveMotors;                     // Left drive motor control
    TalonMotorGroup *               m_pRightDriveMotors;                    // Right drive motor control
    
    // Spike Relays
    Relay *                         m_pLedRelay;                            // Controls whether or not the LEDs are lit up
    
    // Digital I/O
    // (none)
    
    // Analog I/O
    // (none)
    
    // Solenoids
    // Note: No compressor related objects required,
    // instantiating a solenoid gets that for us.
    // (none)
    
    // Servos
    // (none)
    
    // Encoders
    // (none)
    
    // Timers
    Timer *                         m_pAutonomousTimer;                     // Time things during autonomous
    Timer *                         m_pInchingDriveTimer;                   // Keep track of an inching drive operation
    Timer *                         m_pCameraRunTimer;                      // Keep track of how often to do camera intense code runs
    Timer *                         m_pSafetyTimer;                         // Fail safe in case critical operations don't complete
    
    // Accelerometer
    BuiltInAccelerometer *          m_pAccelerometer;                       // Built in roborio accelerometer
    
    // Gyro
    ADXRS450_Gyro *                 m_pAdxrs450Gyro;                        // SPI port FRC gyro
    int                             m_Bno055Angle;                          // Angle from the BNO055 sensor on the RIOduino

    // Camera
    // Note: Only need to have a thread here and tie it to
    // the RobotCamera class, which handles everything else.
    std::thread                     m_CameraThread;
    TriggerChangeValues *           m_pToggleFullProcessingTrigger;
    TriggerChangeValues *           m_pToggleProcessedImageTrigger;

    // Serial port configuration
    static const int                SERIAL_PORT_BUFFER_SIZE_BYTES           = 64;
    static const int                SERIAL_PORT_NUM_DATA_BITS               = 8;
    static const int                SERIAL_PORT_BAUD_RATE                   = 115200;
    static const int                ASCII_0_OFFSET                          = 48;
    const char *                    SERIAL_PORT_PACKET_HEADER               = "Frc120Serial";
    const int                       SERIAL_PORT_PACKET_HEADER_SIZE_BYTES    = sizeof(SERIAL_PORT_PACKET_HEADER);
    char                            m_SerialPortBuffer[SERIAL_PORT_BUFFER_SIZE_BYTES];

    // On board serial port
    SerialPort *                    m_pSerialPort;
    
    // I2C configuration
    std::thread                     m_I2cThread;
    
    // Misc
    RobotMode                       m_RobotMode;                            // Keep track of the current robot state
    Alliance                        m_AllianceColor;                        // Color reported by driver station during a match
    bool                            m_bDriveSwap;                           // Allow the user to push a button to change forward/reverse
    bool                            m_bLed;                                 // Keep track of turning an LED on/off
    
    // CONSTS
    
    // Joysticks/Buttons
    static const ControllerType     DRIVE_CONTROLLER_TYPE                   = CUSTOM_CONTROLLER;
    static const ControllerType     CONTROL_CONTROLLER_TYPE                 = CUSTOM_CONTROLLER;
    
    static const int                DRIVE_JOYSTICK_PORT                     = 0;
    static const int                CONTROL_JOYSTICK_PORT                   = 1;

    // Driver buttons
    static const int                CAMERA_TOGGLE_FULL_PROCESSING_BUTTON    = (DRIVE_CONTROLLER_TYPE == LOGITECH_EXTREME) ? 11 :  7;
    static const int                CAMERA_TOGGLE_PROCESSED_IMAGE_BUTTON    = (DRIVE_CONTROLLER_TYPE == LOGITECH_EXTREME) ? 12 :  8;
    static const int                SELECT_FRONT_CAMERA_BUTTON              = (DRIVE_CONTROLLER_TYPE == LOGITECH_EXTREME) ? 13 :  9;
    static const int                SELECT_BACK_CAMERA_BUTTON               = (DRIVE_CONTROLLER_TYPE == LOGITECH_EXTREME) ? 14 : 10;
    static const int                DRIVE_CONTROLS_FORWARD_BUTTON           = (DRIVE_CONTROLLER_TYPE == LOGITECH_EXTREME) ? 15 : 11;
    static const int                DRIVE_CONTROLS_REVERSE_BUTTON           = (DRIVE_CONTROLLER_TYPE == LOGITECH_EXTREME) ? 16 : 12;
    
    // Control buttons
    static const int                ESTOP_BUTTON                            = (CONTROL_CONTROLLER_TYPE == LOGITECH_EXTREME) ? 14 :  8;

    // CAN Signals
    static const int                LEFT_MOTORS_CAN_START_ID                = 1;
    static const int                RIGHT_MOTORS_CAN_START_ID               = 4;

    // PWM Signals
    // (none)
    
    // Relays
    static const int                LED_RELAY_ID                            = 3;
    
    // Digital I/O Signals
    // (none)
    
    // Analog I/O Signals
    // (none)
    
    // Solenoid Signals
    // (none)
    
    // Misc
    const std::string               AUTO_ROUTINE_1_STRING                   = "Autonomous Routine 1";
    const std::string               AUTO_ROUTINE_2_STRING                   = "Autonomous Routine 2";
    const std::string               AUTO_ROUTINE_3_STRING                   = "Autonomous Routine 3";
    const std::string               AUTO_TEST_ROUTINE_STRING                = "Autonomous Test Routine";

    static const int                OFF                                     = 0;
    static const int                ON                                      = 1;
    static const int                SINGLE_MOTOR                            = 1;
    static const int                NUMBER_OF_LEFT_DRIVE_MOTORS             = 3;
    static const int                NUMBER_OF_RIGHT_DRIVE_MOTORS            = 3;
    static const int                POV_INPUT_TOLERANCE_VALUE               = 30;
    static const int                SCALE_TO_PERCENT                        = 100;
    static const int                QUADRATURE_ENCODING_ROTATIONS           = 4096;
    static const char               NULL_CHARACTER                          = '\0';
    
    static const unsigned           I2C_RUN_INTERVAL_MS                     = 500U;
    
    static constexpr double         JOYSTICK_TRIM_UPPER_LIMIT               =  0.10;
    static constexpr double         JOYSTICK_TRIM_LOWER_LIMIT               = -0.10;
    static constexpr double         CONTROL_THROTTLE_VALUE_RANGE            =  0.65;
    static constexpr double         CONTROL_THROTTLE_VALUE_BASE             =  0.35;
    static constexpr double         DRIVE_THROTTLE_VALUE_RANGE              =  0.65;
    static constexpr double         DRIVE_THROTTLE_VALUE_BASE               =  0.35;
    static constexpr double         DRIVE_MOTOR_UPPER_LIMIT                 =  1.00;
    static constexpr double         DRIVE_MOTOR_LOWER_LIMIT                 = -1.00;
    static constexpr double         DRIVE_WHEEL_DIAMETER_INCHES             =  4.00;
    static constexpr double         INCHING_DRIVE_SPEED                     =  0.25;
    static constexpr double         INCHING_DRIVE_DELAY_S                   =  0.10;
    
    static constexpr double         CAMERA_RUN_INTERVAL_S                   =  1.00;
    static constexpr double         SAFETY_TIMER_MAX_VALUE                  =  5.00;
    
    static const int                SONAR_LED_WARN_DIST_INCHES              = 3;
    static const uint32_t           SONAR_DRIVE_STATE_SIDE_MASK             = 0x0F;
    static const uint32_t           SONAR_DRIVE_STATE_LATERAL_MASK          = 0xF0;
    
};  // End class



////////////////////////////////////////////////////////////////
/// @method YtaRobot::CheckForDriveSwap
///
/// Updates the drive control direction.
///
////////////////////////////////////////////////////////////////
inline void YtaRobot::CheckForDriveSwap()
{
    // Check if the driver pushed the button to have
    // forward be reverse and vice versa
    if ( m_pDriveJoystick->GetRawButton(DRIVE_CONTROLS_FORWARD_BUTTON) )
    {
        m_bDriveSwap = false;
    }
    else if ( m_pDriveJoystick->GetRawButton(DRIVE_CONTROLS_REVERSE_BUTTON) )
    {
        m_bDriveSwap = true;
    }
    else
    {
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::GetThrottleControl
///
/// Returns a throttle value based on input from the joystick.
///
////////////////////////////////////////////////////////////////
inline double YtaRobot::GetThrottleControl(Joystick * pJoystick)
{
    // Get throttle control
    // The z axis goes from -1 to 1, so it needs to be normalized.
    // Subtract one and negate to make it zero based to give a value
    // between zero and two.  This will be used to scale the voltage
    // to the motors.  It essentially computes the max speed value
    // that can be reached.
    if (pJoystick == m_pDriveJoystick)
    {
        return ((pJoystick->GetThrottle() - 1.0) / -2.0) * DRIVE_THROTTLE_VALUE_RANGE + DRIVE_THROTTLE_VALUE_BASE;
    }
    else
    {
        return ((pJoystick->GetThrottle() - 1.0) / -2.0) * CONTROL_THROTTLE_VALUE_RANGE + CONTROL_THROTTLE_VALUE_BASE;
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::GetThrottleControl
///
/// Returns a throttle value based on input from a Logitech
/// Gamepad controller.
///
////////////////////////////////////////////////////////////////
inline double YtaRobot::GetThrottleControl(YtaController * pController)
{
    // Gamepad throttle already comes back between 0 and +1, so no need to normalize.
    return (pController->GetThrottle() * DRIVE_THROTTLE_VALUE_RANGE) + DRIVE_THROTTLE_VALUE_BASE;
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::GetGyroValue
///
/// This method is used to get a value from an analog gyro
/// sensor.  There are three possible places a gyro could be
/// connected: analog sensor, the on board SPI port (ADXRS450),
/// or externally (BNO055 on a RIOduino).  This method will
/// obtain a value from the sensor corresponding to the passed
/// in parameter.
///
////////////////////////////////////////////////////////////////
inline double YtaRobot::GetGyroValue(GyroType gyroType, AnalogGyro * pSensor)
{
    double value = 0.0;
    
    switch (gyroType)
    {
        case ADXRS450:
        {
            value = m_pAdxrs450Gyro->GetAngle();
            break;
        }
        case ANALOG:
        {
            if (pSensor != nullptr)
            {
                value = pSensor->GetAngle();
            }
            break;
        }
        case BNO055:
        {
            // Read the angle
            m_Bno055Angle = RobotI2c::GetGyroData()->m_xAxisInfo.m_Angle;
            
            // Reapply negative sign if needed
            if (RobotI2c::GetGyroData()->m_xAxisInfo.m_bIsNegative)
            {
                m_Bno055Angle *= -1;
            }
            
            //RobotUtils::DisplayFormattedMessage("BNO055 angle: %i\n", m_Bno055Angle);
            value = static_cast<double>(m_Bno055Angle);
            
            break;
        }
        default:
        {
            // Should never happen
            ASSERT(false);
            break;
        }
    }
    
    if (RobotUtils::DEBUG_PRINTS)
    {
        SmartDashboard::PutNumber("Gyro angle", value);
    }
    
    return value;
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::GetSonarSensorValue
///
/// This method is used to get a value from the sonar sensor.
/// It is intended to be used to turn a sensor briefly on and
/// get a reading from it so as to not interfere with other
/// sensors that may need to get readings.
///
////////////////////////////////////////////////////////////////
inline double YtaRobot::GetSonarSensorValue(Ultrasonic * pSensor)
{
    return 0.0;
    
    /*
    pSensor->SetEnabled(true);
    double sensorValue = pSensor->GetRangeInches();
    pSensor->SetEnabled(false);
    return sensorValue;
    */
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::CheckAndUpdateRobotMode
///
/// Checks the current robot mode for a state change and updates
/// accordingly, including displaying a message.
///
////////////////////////////////////////////////////////////////
void YtaRobot::CheckAndUpdateRobotMode(RobotMode robotMode)
{
    // These array messages match the order of the RobotMode enum
    const char * MODE_CHANGE_ENTER_MESSAGES[] = 
                {
                    "Autonomous entered.",
                    "Teleop entered.",
                    "Test entered.",
                    "Disabled entered."
                };

    const char * MODE_CHANGE_EXIT_MESSAGES[] = 
                {
                    "Autonomous exited.",
                    "Teleop exited.",
                    "Test exited.",
                    "Disabled exited."
                };
    
    // Check for the mode to have changed
    if (m_RobotMode != robotMode)
    {
        // First display the exit message for the old mode
        switch (m_RobotMode)
        RobotUtils::DisplayMessage(MODE_CHANGE_EXIT_MESSAGES[m_RobotMode]);

        // Enter the new mode and display an enter message
        m_RobotMode = robotMode;
        RobotUtils::DisplayMessage(MODE_CHANGE_ENTER_MESSAGES[m_RobotMode]);
    }
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::Limit
///
/// This method is used to prevent a value outside the range
/// specified by upper and lower from being sent to physical
/// devices.
///
////////////////////////////////////////////////////////////////
inline double YtaRobot::Limit( double num, double upper, double lower )
{
    if ( num > upper )
    {
        return upper;
    }
    else if ( num < lower )
    {
        return lower;
    }

    return num;
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::Trim
///
/// This method is used to ensure a signal value is above a
/// certain threshold to ensure there is actual input to a
/// device and not just noise/jitter.
///
////////////////////////////////////////////////////////////////
inline double YtaRobot::Trim( double num, double upper, double lower )
{
    if ( (num < upper) && (num > lower) )
    {
        return 0;
    }
    
    return num;
}



////////////////////////////////////////////////////////////////
/// @method YtaRobot::TriggerChangeValues::DetectChange
///
/// This method is used to check if a button has undergone a
/// state change.  The same button can be used to reverse state
/// of a particular part of the robot (such as a motor or
/// solenoid).  If the state is reversed inside an 'if'
/// statement that is part of a loop, the final state will be
/// whatever transition just occurred, which could be back to
/// the same state started in.  The intended use case is to
/// have TriggerChangeValues variables in the code and update
/// their 'current' value each time through the loop by reading
/// the joystick input.  This input will then be checked against
/// the old input and return 'true' if it detects the button has
/// been pressed.  This method is intended to be called inside
/// 'if' statements for logic control.
///
////////////////////////////////////////////////////////////////
inline bool YtaRobot::TriggerChangeValues::DetectChange()
{
    // Only report a change if the current value is different than the old value
    // Also make sure the transition is to being pressed since we are detecting
    // presses and not releases
    if ( (this->m_bCurrentValue != this->m_bOldValue) && this->m_bCurrentValue )
    {
        // Update the old value, return the button was pressed
        this->m_bOldValue = this->m_bCurrentValue;
        return true;
    }
    
    // Otherwise update the old value
    this->m_bOldValue = this->m_bCurrentValue;
    return false;
}

#endif // YTAROBOT_HPP
