/** @file       IRTrackerUtils.h
 *  @brief      Function prototypes for general utilities used by the Holo2IRTracker class. 
 *              Implementations split in other files by namespace.
 *
 *  @author     Hisham Iqbal
 *  @copyright  &copy; 2023 Hisham Iqbal
 */

#ifndef IR_TRACKER_UTILS_H
#define IR_TRACKER_UTILS_H

#include <Eigen/Dense>
#include "opencv2/core.hpp"
#include <vector>
#include <map>

/**
 * @namespace   IRTrackerUtils
 * @brief       Utility namespace for various functions used by the \ref Holo2IRTracker class 
*/
namespace IRTrackerUtils
{
    //-------------------------------------------------------------------------------------------------------------
    //! @struct InfraBlobInfo
    //! @brief Data stored about any valid blobs seen in image
    //! 
    //! Qualifying criteria: circularity/area/raw depth-value
    struct InfraBlobInfo
    {
        cv::Point2f         PixelCoordinate;    /*!< 2D location of blob, stored for label purposes */
        Eigen::Vector3d     DepthLocation;      /*!< 3D location of this blob in the sensor coordinate frame */
        Eigen::Vector3d     WorldLocation;      /*!< 3D location of this blob in the world coordinate frame */
    };
    //-------------------------------------------------------------------------------------------------------------

    //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    //! @struct TrackedTool
    //! @brief Struct assigned once a tool has been detected and can be stored in an internal tool map/dictionary
    struct TrackedTool
    {
        uint8_t                         ID = 255;                   /*!< 8-bit tool ID, should be uniquely assigned */
        bool                            VisibleToHoloLens = false;  /*!< Flag whether tool visible in last seen frame */
        std::vector<Eigen::Vector3d>    GeometryPoints;             /*!< Known coordinates of tool - right-handed marker positions (from CAD/config files) */
        std::vector<Eigen::Vector3d>    ObservedPoints_World;       /*!< Marker positions defined in world-frame (defined as startup pose)*/
        std::vector<Eigen::Vector3d>    ObservedPoints_Depth;       /*!< Observed tool marker points in depth - sensor frame (should be the same order as GeometryPoints) */
        Eigen::Matrix4d                 PoseMatrix_HoloWorld;       /*!< 4x4 transform matrix of tool pose in world frame */
        Eigen::Matrix4d                 PoseMatrix_DepthCamera;     /*!< 4x4 transform matrix of tool pose w.r.t depth sensor frame*/
        std::vector<cv::Point2i>        ObservedImgKeypoints;       /*!< Image coordinates for marker-centres for labelling (same order as GeometryPoints) */
    };
    //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //! Standard map structure used in this set of utils, used to contain information about tools of interest equipped with IR reflective markers
    typedef std::map<uint8_t, TrackedTool> ToolDictionary;

    //! Function pointer which mimics the signature of the Research Mode API's MapImagePointToUnitPlane function 
    typedef std::function<bool(float(&)[2], float(&)[2])> UnmapFunction;
}

//! @namespace IRTrackerUtils::JSONUtils
//! @brief Namespace for handling config files passed into this application
namespace IRTrackerUtils::JSONUtils
{
    //! @brief Helper function which takes in a JSON compliant string to initialize a ToolDictionary
    //! 
    //! Example structure from outer to inner layers:
    //! A main key called "tools," containing an array of mini 'tool-structs' inside braces {}.
    //! Each {} struct inside the "tools" array should contain (1) name (2) id (integer)
    //! and (3) an array of xyz coordinates (in meters and right-handed).
    //! 
    //! {"tools": [\n
    //! {"name": "Probe",\n
    //!     "id": 1,\n
    //!     "coordinates":\n 
    //!     [["0.001", "0.002", "0.003"],\n
    //!     ["0.000", "0.002", "0.003"],\n
    //!     ["0.001", "0.002", "0.000"],\n
    //!     ["0.000", "0.000", "0.000"]]}\n
    //! }
    //! 
    //! @param jsonString        JSON formatted string following the above convention 
    //! @param toolDictionary    ToolDictionary to be populated  
    void FillToolDictionaryFromJSONString(const std::string jsonString, std::map<uint8_t, TrackedTool>& toolDictionary);
}

//! @namespace   IRTrackerUtils::ImageProc 
//! @brief       Various utility functions for doing image processing on data retrieved from the HoloLens 2's AHAT depth sensor 
namespace IRTrackerUtils::ImageProc
{
    //-------------------------------------------------------------------------------------------------------------
    //! Current implementations of BlobDetection method labels
    enum class BlobDetectionMethod 
    {
        Basic,              ///< Basic contour-detection filtering by thresholding, area and circularity 
                            ///< using cv::findContours
        RefineByScaling     ///< Builds on the BlobDetectionMethod::Basic by enlarging the blob region to try 
                            ///< a sub-pixel refinement of centre-estimation.
    };
    //-------------------------------------------------------------------------------------------------------------

    //-------------------------------------------------------------------------------------------------------------
    //! @brief   Template function for converting native arrays to the cv::Mat type 
    //! 
    //! @tparam T        Currently supporting uint16_t/uint8_t per HL2 AHAT sensor image types
    //! @param src       Input native array 
    //! @param dst       Output cv::Mat array 
    //! @param rows      Input size parameter 
    //! @param cols      Input size parameter 
    //! @param chs       Assumed 1 for grayscale image 
    //-------------------------------------------------------------------------------------------------------------

    //-------------------------------------------------------------------------------------------------------------
    template <typename T> void NativeToCVMat(const T* src, cv::Mat& dst, int rows, int cols);        
    // explicitly instantiate the templates we are likely to use to avoid linker errors
    template void NativeToCVMat(const uint16_t* src, cv::Mat& dst, int rows, int cols);
    template void NativeToCVMat(const uint8_t* src, cv::Mat& dst, int rows, int cols);
    //-------------------------------------------------------------------------------------------------------------
        
    //-------------------------------------------------------------------------------------------------------------
    //! @brief   Generic method for populating a vector of pixel-locations indicating centres of any detected
    //!          blobs in \p processedImage. Internal implementation routes what to do based on \p method 
    //! 
    //! @param processedImage        Expecting an 8-bit image with a reasonable dynamic range 
    //! @param method                Choose from implemented methods for blob detection 
    //! @param outPixelLocations     Vector to be filled with pixel locations of detected blob centres
    void DetectBlobs2D(cv::Mat& processedImage, BlobDetectionMethod method, std::vector<cv::Point2f>& outPixelLocations);
    //-------------------------------------------------------------------------------------------------------------
        
    //-------------------------------------------------------------------------------------------------------------
    //! @brief   Constructs a vector of valid \ref InfraBlobInfo after cross-checking the depth values
    //! 
    //!  Iterates over the vector \p inblobPixels2D to check depth values. 2D pixel to 3D vector conversion done via 
    //!  the \p MapImagePointToUnitPlane function. \p outBlobInfo should then contain a list of valid 3D blobs which
    //!  are likely to belong to infrared markers.
    //! 
    //! @param inDepthImg                    16-bit depth image from HL2 without any processing required 
    //! @param inDepth2World                 Transform matrix of depth sensor to world coordinate system in this frame 
    //! @param inblobPixels2D                Detected blob pixel locations (2D) to iterate over
    //! @param MapImagePointToUnitPlane      Pointer to function that converts from 2D pixel locations (u,v) to the camera's unit plane (x,y,1) 
    //! @param outBlobInfo                   Vector to populate with valid 3D blob info specified by \ref InfraBlobInfo
    void ValidateBlobs3D(
        const cv::Mat&                       inDepthImg, 
        const Eigen::Ref<Eigen::Matrix4d>    inDepth2World,
        const std::vector<cv::Point2f>&      inblobPixels2D, 
        const UnmapFunction                  MapImagePointToUnitPlane, 
        std::vector<InfraBlobInfo>&          outBlobInfo
    );
    //-------------------------------------------------------------------------------------------------------------
        
    //-------------------------------------------------------------------------------------------------------------
    //! @brief Helper function to increase dynamic range of image and then convert to an 8-bit image
    //! 
    //!  Internally scales each pixel by factor of 64 to 'brighten' and handles conversion from 16 to 8bit 
    //! 
    //! @param inputRaw16BitImg      Expecting 16 bit AB image as obtained from HL2 AHAT depth sensor
    //! @param output8BitImg         A processed 8-bit AB image with an increased dynamic range 
    void RebalanceImgAnd8Bit(cv::Mat& inputRaw16BitImg, cv::Mat& output8BitImg);
    //-------------------------------------------------------------------------------------------------------------
        
    //-------------------------------------------------------------------------------------------------------------
    //! @brief  Helper function to add annotations on \p Img2Label to draw crosses at any detected tool's marker centres
    //! 
    //! @param toolDictionary 
    //! @param Img2Label 
    void LabelImageWithToolDictData(const std::map<uint8_t, IRTrackerUtils::TrackedTool>& toolDictionary, cv::Mat Img2Label);
    //-------------------------------------------------------------------------------------------------------------
        
    //-------------------------------------------------------------------------------------------------------------
    //! @brief  Helper function which creates an 8-bit processed image of the depth map created by ResearchModeAPI
    //! 
    //! Values above 4090 are floored to 0, and all other 16-bit values are linearly mapped to the 8-bit range.
    //! 
    //! @param input16bitdepthImg   Raw 16-bit depth image from ResearchModeAPI without any processing 
    //! @param output8bitDepth      A ready to display image 
    void GetProcessed8BitDepthImg(const cv::Mat& input16bitdepthImg, cv::Mat& output8bitDepth);
    //-------------------------------------------------------------------------------------------------------------
        
    //-------------------------------------------------------------------------------------------------------------
    //! @brief   Helper function to return interpolated image value in a grayscale image based on float indices
    //! 
    //! @param image     Image to read 
    //! @param point     cv::Point2f struct to support non-integer indices 
    //! @return          Interpolated value based on bilinear interpolation of 4 surrounding pixels 
    float BilinearInterpolation(const cv::Mat& image, const cv::Point2f& point);
    //-------------------------------------------------------------------------------------------------------------
}

#endif // IR_TRACKER_UTILS_H