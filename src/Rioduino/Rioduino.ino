////////////////////////////////////////////////////////////////////////////////
/// @file   Rioduino.hpp
/// @author David Stalter
///
/// @details
/// This is the class declaration and implementation of functionality on the
/// RIOduino for a FRC robot.  It handles offloaded sensors/peripherals and
/// communicates to the roboRIO via I2C on the MXP port.
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// INCLUDES
#include <Adafruit_Sensor.h>                  // for base sensor support
#include <Adafruit_BNO055.h>                  // for BNO055 library
#include <Wire.h>                             // for I2C communication
#include "RoborioRioduinoSharedData.hpp"      // for shared data structures


////////////////////////////////////////////////////////////////////////////////
/// Class: YtaRobot
///
/// Class that contains the declarations and functions for running the RIOduino.
///
////////////////////////////////////////////////////////////////////////////////
class YtaRioduino
{
public:
  
  // Wrappers for the setup/loop Arduino functions
  static void Initialize();
  static void Run();
  
private:
  
  // Makes for easier use/readability of interacting with I2cData
  typedef RoborioRioduinoSharedData::I2cData I2cData;

  // Support function for displaying a message if prints are enabled
  template <typename T>
  inline static void DisplayMessage(T message)
  {
    if (DEBUG_PRINTS)
    {
      Serial.println(message);
    }
  }
  
  // Support function to show the RIOduino is still executing
  inline static void HeartBeat()
  {
    static int heartBeat = 1;
    if (DEBUG_PRINTS)
    {
      Serial.print("HeartBeat: ");
      Serial.println(heartBeat++);
    }
  }
  
  // Member functions
  static void I2cOnReceive(int bytesReceived);
  static void I2cOnRequest();
  static void BuildI2cData();
  static void GyroSequence(bool bReadNewCenter);
  
  // Member variables
  static Adafruit_BNO055 m_Bno;
  static I2cData m_I2cData;
  
  static double m_RobotAngle;
  static double m_RobotAbsoluteAngle;
  static double m_RobotRelativeAngle;
  static double m_RobotCenterPoint;
  
  // Constants
  static const int          DEBUG_RED_LED_PIN           = 6;
  static const int          DEBUG_GREEN_LED_PIN         = 7;
  static const int          BNO055_SENSOR_ID            = 55;
  static const int          BNO055_SAMPLE_RATE_MS       = 75;
  static const int          HEART_BEAT_RATE_MS          = 1000;
  static const unsigned int I2C_HEADER_DATA             = 0x0120;
  static const unsigned int I2C_FOOTER_DATA             = 0x0210;
  static constexpr double   ONE_HUNDRED_EIGHTY_DEGREES  = 180.0;
  static constexpr double   THREE_HUNDRED_SIXTY_DEGREES = 360.0;

  static const bool         DEBUG_PRINTS                = false;
  static const bool         DEBUG_I2C_TRANSACTIONS      = false;
  static const bool         DEBUG_GYRO_READINGS         = false;

  // Constructor, destructor, copy, assignment
  YtaRioduino();
  ~YtaRioduino();
  YtaRioduino(const YtaRioduino &);
  YtaRioduino & operator=(const YtaRioduino &);
};

// STATIC MEMBER DATA
Adafruit_BNO055 YtaRioduino::m_Bno = Adafruit_BNO055(BNO055_SENSOR_ID);
double YtaRioduino::m_RobotAngle = 0.0;
double YtaRioduino::m_RobotAbsoluteAngle = 0.0;
double YtaRioduino::m_RobotRelativeAngle = 0.0;
double YtaRioduino::m_RobotCenterPoint = 0.0;
YtaRioduino::I2cData YtaRioduino::m_I2cData;


////////////////////////////////////////////////////////////////////////////////
/// Method: setup
///
/// Details:  The Arduino initialization function called during controller
///           start up. 
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  YtaRioduino::Initialize();
}


////////////////////////////////////////////////////////////////////////////////
/// Method: loop
///
/// Details:  The continuous Arduino background user loop.
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  YtaRioduino::Run();
}


////////////////////////////////////////////////////////////////////////////////
/// Method: Initialize
///
/// Details:  Initializes data/peripherals for the RIOduino.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::Initialize()
{
  // Start up the serial port
  Serial.begin(115200);
  DisplayMessage("FRC 120 RIOduino.");
  
  // Configure debug pins
  pinMode(DEBUG_RED_LED_PIN, OUTPUT);
  pinMode(DEBUG_GREEN_LED_PIN, OUTPUT);
  
  // Open the I2C port
  Wire.begin(RoborioRioduinoSharedData::I2C_DEVICE_ADDRESS);
  Wire.onReceive(I2cOnReceive);
  Wire.onRequest(I2cOnRequest);
  
  // Clear I2C data and set constant fields
  memset(&m_I2cData, 0U, sizeof(m_I2cData));
  m_I2cData.m_Header = I2C_HEADER_DATA;
  m_I2cData.m_Footer = I2C_FOOTER_DATA;
  
  // Initialize the 9-axis sensor
  while (!m_Bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    DisplayMessage("No BNO055 detected... check wiring or I2C ADDR!");

    // Delay before trying again
    const unsigned int ONE_SECOND_DELAY_MS = 1000;
    delay(ONE_SECOND_DELAY_MS);
  }
  
  m_Bno.setExtCrystalUse(true);
}


////////////////////////////////////////////////////////////////////////////////
/// Method: Run
///
/// Details:  Main loop for the RIOduino program.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::Run()
{
  // Get new information from the sensor
  GyroSequence(false);
  
  // Build an I2C response packet
  BuildI2cData();
  
  // Update heartbeat
  static unsigned int lastHeartBeatTimeMs = 0U;
  unsigned int currentTimeMs = millis();
  if ((currentTimeMs - lastHeartBeatTimeMs) > HEART_BEAT_RATE_MS)
  {
    HeartBeat();
    lastHeartBeatTimeMs = currentTimeMs;
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: I2cOnReceive
///
/// Details:  Receives I2C data from the roboRIO.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::I2cOnReceive(int bytesReceived)
{
  if (DEBUG_I2C_TRANSACTIONS)
  {
    Serial.print("On receive: ");
    
    for (int i = 0; i < bytesReceived; i++)
    {
      const char c = Wire.read();
      Serial.print(c);
    }
    
    Serial.println();
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: I2cOnRequest
///
/// Details:  Sends I2C data to the roboRIO.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::I2cOnRequest()
{
  Wire.write(reinterpret_cast<byte *>(&m_I2cData), sizeof(m_I2cData));

  if (DEBUG_I2C_TRANSACTIONS)
  {
    Serial.print("On request: ");
    const byte * pData = reinterpret_cast<byte *>(&m_I2cData);
    for (size_t i = 0; i < sizeof(m_I2cData); i++)
    {
      Serial.print(*pData++, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Method: BuildI2cData
///
/// Details:  Builds the data to send over to the roboRIO.  Note the I2C request
///           could come in the middle of this, so it is up to the roboRIO side
///           to validate the data set.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::BuildI2cData()
{
  // Indicate gyro data is being sent over
  m_I2cData.m_DataSelection = RoborioRioduinoSharedData::I2cDataSelection::GYRO_DATA;
  
  // First get the robot angle since it will be manipulated before sending
  double robotAngle = m_RobotAngle;
  
  // Check if it's negative
  if (robotAngle < 0.0)
  {
    digitalWrite(DEBUG_RED_LED_PIN, HIGH);
    digitalWrite(DEBUG_GREEN_LED_PIN, LOW);
    
    m_I2cData.m_DataBuffer.m_GyroData.m_xAxisInfo.m_bIsNegative = true;
    
    // For simplicity with the different architectures, always send a positive angle
    robotAngle *= -1.0;
  }
  else
  {
    digitalWrite(DEBUG_RED_LED_PIN, LOW);
    digitalWrite(DEBUG_GREEN_LED_PIN, HIGH);
    
    m_I2cData.m_DataBuffer.m_GyroData.m_xAxisInfo.m_bIsNegative = false;
  }
  
  // Set the angle, deliberately converting to an integer value
  m_I2cData.m_DataBuffer.m_GyroData.m_xAxisInfo.m_Angle = static_cast<uint16_t>(round(robotAngle));
}


////////////////////////////////////////////////////////////////////////////////
/// Method: GyroSequence
///
/// Details:  Reads information from the BNO055 9-axis sensor.
////////////////////////////////////////////////////////////////////////////////
void YtaRioduino::GyroSequence(bool bReadNewCenter)
{
  static unsigned int lastGyroTimeMs = 0U;
  unsigned int currentTimeMs = millis();
  
  if ((currentTimeMs - lastGyroTimeMs) > BNO055_SAMPLE_RATE_MS)
  {
    // For normalization to arbitrary angle as center:
    //
    // Defaults to zero
    // double m_CenterSetPointDegrees = 0.0;
    // m_RobotAbsoluteAngle = bnoSensorEvent.orientation.x;
    // m_RobotRelativeAngle = m_RobotAbsoluteAngle - m_CenterSetPointDegrees;
    // if (m_RobotRelativeAngle < 0.0)
    // {
    //   // Negative angles must be normalized from 360 (the addition here is actually subtraction)
    //   m_RobotRelativeAngle += THREE_HUNDRED_SIXTY_DEGREES;
    // }
    
    // Get a new sensor event
    sensors_event_t bnoSensorEvent;
    m_Bno.getEvent(&bnoSensorEvent);
    
    // Save off a new center if one was requested
    if (bReadNewCenter)
    {
      DisplayMessage("Reading new center...");
      m_RobotCenterPoint = bnoSensorEvent.orientation.x;
    }
    
    // The absolute angle is what the sensor reports.
    // The relative angle needs to be computed from the absolute angle.
    m_RobotAbsoluteAngle = bnoSensorEvent.orientation.x;
    m_RobotRelativeAngle = m_RobotAbsoluteAngle - m_RobotCenterPoint;
    
    // The relative angle can be anywhere from -360 to + 360 at this point.
    // We need an angle between 0 -> 360 only.
    // Negative angles must be normalized from 360 (the addition here is actually subtraction).
    if (m_RobotRelativeAngle < 0.0)
    {
      m_RobotRelativeAngle += THREE_HUNDRED_SIXTY_DEGREES;
    }
    
    // Now that we have an angle from 0 -> 360, convert it to -180 -> 180
    if (m_RobotRelativeAngle > ONE_HUNDRED_EIGHTY_DEGREES)
    {
      // convert left half of unit circle to negative
      // old: 360 (top) -> 270 (left) -> 180 (bottom)
      // new: 0 (top) -> -90 (left) -> -180 (bottom)
      m_RobotRelativeAngle -= THREE_HUNDRED_SIXTY_DEGREES;
    }
    
    // Robot angle is the relative angle
    m_RobotAngle = m_RobotRelativeAngle;
    
    // Update the last time a reading was taken
    lastGyroTimeMs = currentTimeMs;
    
    if (DEBUG_GYRO_READINGS)
    {
      Serial.print("Absolute angle: ");
      Serial.println(m_RobotAbsoluteAngle);
      Serial.print("Relative angle: ");
      Serial.println(m_RobotRelativeAngle);
    }
  }
}

