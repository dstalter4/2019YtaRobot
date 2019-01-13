////////////////////////////////////////////////////////////////////////////////
/// @file   RobotCamera.hpp
/// @author David Stalter
///
/// @details
/// A class designed to support camera functionality on the robot.
///
/// Copyright (c) 2019 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/WPILib.h"                     // for FRC library
#include "cameraserver/CameraServer.h"      // for camera support
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/types.hpp"

// C++ INCLUDES
// (none)

using namespace frc;

////////////////////////////////////////////////////////////////
/// @class RobotCamera
///
/// Class that provides methods for interacting with the camera.
///
////////////////////////////////////////////////////////////////
class RobotCamera
{
public:
    
    enum CameraType
    {
        FRONT_USB,
        BACK_USB,
        AXIS
    };
    
    // Set whether or not full vision processing can occur
    inline static void SetFullProcessing(bool bState);
    
    // Pick a camera to use
    inline static void SetCamera(CameraType camera);
    
    // Toggle between cameras
    inline static void ToggleCamera();
    
    // Toggle between what processed image is shown on the dashboard
    static void ToggleCameraProcessedImage();
    
    // The vision thread itself
    static void VisionThread();

private:
    
    // Update values on the SmartDashboard
    static void UpdateSmartDashboard();

    // Process a mat through the vision pipeline
    static void ProcessImage();
    
    // Specific operations the vision pipeline will perform
    static void FilterImageHsv();
    static void ErodeImage();
    static void FindContours();
    static void FilterContours();

    // Process the filtered contours to find the reflective tape
    static void FindReflectiveTapeTarget();

    // Compute some useful information about the reflective tape
    static void CalculateReflectiveTapeValues();

    // Constructor
    RobotCamera();

    // Destructor, copy constructor, assignment operator
    ~RobotCamera();

    RobotCamera(const RobotCamera &) = delete;
    RobotCamera & operator=(const RobotCamera &) = delete;
    
    // MEMBER VARIABLES
    
    // A structure to hold measurements of a contour
    struct VisionTargetReport
    {
        double m_BoundingRectX;             // Bounding rectangle top left corner X
        double m_BoundingRectY;             // Bounding rectangle top left corner Y
        double m_BoundingRectWidth;         // Bounding rectangle width
        double m_BoundingRectHeight;        // Bounding rectangle height
        double m_BoundingRectArea;          // Bounding rectangle area
        double m_BoundingRectAspectRatio;   // Bounding rectangle aspect ratio
        
        double m_Area;                      // Contour area
        double m_Perimeter;                 // Contour perimeter
        double m_ConvexHullArea;            // Contour convex hull area
        double m_Solidity;                  // Contour solidity
        double m_Vertices;                  // Contour vertices count

        double m_PercentAreaToImageArea;    // Percentage of the area the contour occupies
        double m_TrapezoidPercent;          // Likelihood that this is a true rectangle
        double m_CameraDistanceX;           // Distance to the target from the camera, measured by width
        double m_CameraDistanceY;           // Distance to the target from the camera, measured by height
        double m_GroundDistance;            // Actual ground distance to the target
        bool   m_bTargetInRange;            // Remember the last result from full vision processing
        bool   m_bIsValid;                  // Indicates if the current report is valid
    };
    
    // Camera, sinks, sources
    static cs::UsbCamera                        m_Cam0;                             // USB camera 0
    static cs::CvSink                           m_Cam0Sink;                         // Sink for camera 0
    static cs::UsbCamera                        m_Cam1;                             // USB camera 1
    static cs::CvSink                           m_Cam1Sink;                         // Sink for camera 1
    static cs::CvSource                         m_CameraOutput;                     // Output source for processed images
    
    // Mats
    static cv::Mat                              m_SourceMat;                        // The originating source mat from the current camera
    static cv::Mat                              m_ResizeOutputMat;                  // Resized source mat
    static cv::Mat                              m_HsvThresholdOutputMat;            // HSV filtered mat
    static cv::Mat                              m_ErodeOutputMat;                   // Erode output mat
    static cv::Mat                              m_ContoursMat;                      // Contours output mat
    static cv::Mat                              m_FilteredContoursMat;              // Filtered contours output mat
    static cv::Mat                              m_VisionTargetMat;                  // The best candidate vision target mat
    static cv::Mat *                            m_pDashboardMat;                    // Pointer to which mat should currently be sent to the dashboard
    
    // Image artifacts represented by std::vector
    static std::vector<std::vector<cv::Point>>  m_Contours;                         // Contours in the image
    static std::vector<std::vector<cv::Point>>  m_FilteredContours;                 // Filtered contours in the image
    
    // Misc
    static std::vector<VisionTargetReport>      m_ContourTargetReports;             // Stores information about the contours currently visible
    static VisionTargetReport                   m_VisionTargetReport;               // Information about the vision target
    static CameraType                           m_Camera;                           // Keep track of the current camera to process information from
    static bool                                 m_bDoFullProcessing;                // Indicates whether or not full image processing should occur
    static int                                  m_HeartBeat;                        // Keep alive with the C++ dashboard
    
    // CONSTANTS
    
    static const char *                         CAMERA_OUTPUT_NAME;
    //static const int                            CAMERA_0_DEV_NUM                    = 0;
    //static const int                            CAMERA_1_DEV_NUM                    = 1;
    static const int                            CAMERA_X_RES                        = 320;
    static const int                            CAMERA_Y_RES                        = 240;
    
    static constexpr double                     TARGET_WIDTH_INCHES                 =  2.0;
    static constexpr double                     TARGET_HEIGHT_INCHES                = 16.0;
    static constexpr double                     TARGET_HEIGHT_FROM_GROUND           =  2.0;
    //static constexpr double                     TARGET_MIN_AREA_PERCENT             = 0.0;
    //static constexpr double                     TARGET_MAX_AREA_PERCENT             = 100.0;
    //static constexpr double                     TARGET_RANGE_MIN                    = 132.0;
    //static constexpr double                     TARGET_RANGE_MAX                    = 192.0;
    //static constexpr double                     GROUND_DISTANCE_TOLERANCE           = 6.0;
    static constexpr double                     CAMERA_FOV_DEGREES                  = 50.0;
    static constexpr double                     CAMERA_DIAGONAL_FOV_DEGREES         = 78.0;
    static constexpr double                     CALIBRATED_CAMERA_ANGLE             = 21.5778173;
    static constexpr double                     DEGREES_TO_RADIANS                  = M_PI / 180.0;
    static constexpr double                     DECIMAL_TO_PERCENT                  = 100.0;
};



////////////////////////////////////////////////////////////////
/// @method RobotCamera::SetFullProcessing
///
/// This method sets whether or not full vision processing
/// should occur.
///
////////////////////////////////////////////////////////////////
inline void RobotCamera::SetFullProcessing(bool bState)
{
    m_bDoFullProcessing = bState;
    
    if (!m_bDoFullProcessing)
    {
        // If processing was previously enabled,
        // need to switch back to the default mat.
        m_pDashboardMat = &m_SourceMat;
        SmartDashboard::PutString("Camera Output", "Default");
    }
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::SetCamera
///
/// This method sets which camera is active.
///
////////////////////////////////////////////////////////////////
inline void RobotCamera::SetCamera(CameraType camera)
{
    m_Camera = camera;
}



////////////////////////////////////////////////////////////////
/// @method RobotCamera::ToggleCamera
///
/// This method toggles between which camera is active.
///
////////////////////////////////////////////////////////////////
inline void RobotCamera::ToggleCamera()
{
    m_Camera = (m_Camera == FRONT_USB) ? BACK_USB : FRONT_USB;
}
