////////////////////////////////////////////////////////////////////////////////
/// @file   RoborioRioduinoSharedData.hpp
/// @author David Stalter
///
/// @details
/// This file contains shared data structures between the roboRIO and the
/// RIOduino.
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef ROBORIORIODUINOSHAREDDATA_HPP
#define ROBORIORIODUINOSHAREDDATA_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
// (none)

#define PACKED __attribute__((packed))


////////////////////////////////////////////////////////////////
/// @namespace RoborioRioduinoSharedData
///
/// Namespace that contains declarations for the shared data
/// structures between the roboRIO and RIOduino.  Sizes of
/// members are deliberately set due to the differences in the
/// underlying architectures.
///
////////////////////////////////////////////////////////////////
namespace RoborioRioduinoSharedData
{
    static const int I2C_DEVICE_ADDRESS = 4U;
        
    // Controls which data structure is active in a transfer
    enum I2cDataSelection : uint8_t
    {
        SONAR_DATA = 1,
        GYRO_DATA = 2
    };
    
    // Data structure for transferring sonar data
    struct SonarI2cData
    {
        struct Distances
        {
            uint8_t m_SonarA;
            uint8_t m_SonarB;
        };

        Distances m_FrontDistances;
        Distances m_LeftDistances;
        Distances m_BackDistances;
        Distances m_RightDistances;
    };
    
    // Data structure for transferring BNO055 gyro data
    struct GyroI2cData
    {
        struct AxisInfo
        {
            uint16_t m_Angle;
            uint8_t m_bIsNegative;
        };
        
        AxisInfo m_xAxisInfo;
        AxisInfo m_yAxisInfo;
        AxisInfo m_zAxisInfo;

        static const uint16_t MAX_VALID_ANGLE_VALUE = 180U;
    };
    
    // Represents the I2C data that will be transferred
    struct I2cData
    {
        typedef uint16_t HeaderFooterType;
        
        // From debugging, it looks like I2C transactions are limited to 32 bytes
        static const uint16_t I2C_DATA_EXPECTED_SIZE = 32U;
        static const uint16_t I2C_DATA_BUFFER_SIZE_BYTES = I2C_DATA_EXPECTED_SIZE - (2 * sizeof(HeaderFooterType)) - sizeof(I2cDataSelection);
        
        // Represents different possible ways of interpreting the data buffer
        union PACKED DataBuffer
        {
            SonarI2cData    m_SonarData;
            GyroI2cData     m_GyroData;
            uint8_t         m_RawBuffer[I2C_DATA_BUFFER_SIZE_BYTES];
        };
        
        // Make sure the compiler created the shared buffer correctly
        static_assert(sizeof(DataBuffer) == I2C_DATA_BUFFER_SIZE_BYTES, "I2C buffer wrong size.");
        
        // The members of the I2C transfer data structure
        HeaderFooterType    m_Header;
        I2cDataSelection    m_DataSelection;
        DataBuffer          m_DataBuffer;
        HeaderFooterType    m_Footer;
        
        static const HeaderFooterType I2C_HEADER_DATA = 0x0120;
        static const HeaderFooterType I2C_FOOTER_DATA = 0x0210;
        
        // Make sure the overlayed data structures fit in the space set aside
        static_assert(sizeof(SonarI2cData) <= I2C_DATA_BUFFER_SIZE_BYTES, "I2C buffer not big enough for sonar data.");
        static_assert(sizeof(GyroI2cData)  <= I2C_DATA_BUFFER_SIZE_BYTES, "I2C buffer not big enough for gyro data.");
    };
    
    // Make sure the overall data structure has the correct size
    static_assert(sizeof(I2cData) == I2cData::I2C_DATA_EXPECTED_SIZE, "I2C data does not equal expected size.");
}

#endif // ROBORIORIODUINOSHAREDDATA_HPP
