#include "pch.h"
#include "Holo2IRTracker.h"
#include "CorrespondenceMatcher.h"
#include "Eigen/Dense"
#include <vector>
#include <map>
#include <opencv2/core.hpp>   
#include <functional>
#include "Shiny.h"

/**
 * @file        Holo2IRTracker.cpp
 * @brief       Implementations for \ref Holo2IRTracker
 * @author      Hisham Iqbal
 * @copyright   &copy; Hisham Iqbal 2023
 *
 */


// Define some consts
// AHAT Resolution
constexpr int IMG_WIDTH = 512;
constexpr int IMG_HEIGHT = 512;

constexpr bool USE_REFINED_BLOB_DETECT = false;

namespace // Anonymous Helper Functions
{
    //! @brief Walk through \p validBlobData to figure out if there are any blobs corresponding to tools in the \p toolDictionary
    //! @param validBlobData    Info about blobs detected in the latest frame
    //! @param toolDictionary   Tool dictionary that we will transform if there are any blobs from tools stored in the dictionary
    void TryUpdatingToolDictionary(std::vector<IRTrackerUtils::InfraBlobInfo>& validBlobData, std::map<uint8_t, IRTrackerUtils::TrackedTool>& toolDictionary)
    {
        PROFILE_BLOCK(ToolDictionaryUpdate);
        using namespace Eigen;
        using namespace IRTrackerUtils;
        // vectorise validBlobData members for easy access later
        // cache these vectors?
        std::vector<Vector3d> collectedPoints; collectedPoints.reserve(validBlobData.size());
        std::vector<Vector3d> collectedDepthPoints; collectedDepthPoints.reserve(validBlobData.size());
        std::vector<cv::Point2i> blobPixelLocations; blobPixelLocations.reserve(validBlobData.size());

        for (const InfraBlobInfo& blob : validBlobData)
        {
            // used for registration to holographic world frame
            collectedPoints.emplace_back(blob.WorldLocation);

            // stored in case we do any depth cam specific calculations
            collectedDepthPoints.emplace_back(blob.DepthLocation);

            // stored for image labelling purpose
            blobPixelLocations.emplace_back(blob.PixelCoordinate);
        }

        typedef std::vector<std::vector<int>> ConfigurationList;
        for (auto& [_, tool] : toolDictionary)
        {
            // initialise / zero appropriate values
            tool.PoseMatrix_HoloWorld = Eigen::Matrix4d::Identity();
            tool.VisibleToHoloLens = false;
            tool.ObservedImgKeypoints.clear();
            tool.ObservedPoints_Depth.clear();
            tool.ObservedPoints_World.clear();

            ConfigurationList candidateList;
            PROFILE_BEGIN(FindingPointCorrespondence);
            bool toolNotFound = !CorrespondenceMatcher::GetPointCorrespondence(tool.GeometryPoints, collectedPoints, candidateList);
            PROFILE_END();
            if (toolNotFound) { continue; }

            if (candidateList.size() == 0) { continue; } // this shouldn't happen, but just in case
            std::vector<int> indexList = candidateList[0];

            /**
             * for loop which correctly assigns and orders observed points so it
             * matches the correspondence order of the reference points
             */
            for (const auto& idx : indexList)
            {
                if (idx > -1 && idx < collectedPoints.size())
                {
                    tool.ObservedPoints_World.emplace_back(collectedPoints[idx]);
                    tool.ObservedPoints_Depth.emplace_back(collectedDepthPoints[idx]);
                    tool.ObservedImgKeypoints.emplace_back(blobPixelLocations[idx]);
                }
            }

            // this shouldn't be the case, but just for safety
            if (tool.GeometryPoints.size() != tool.ObservedPoints_World.size()) continue;

            // gives us the transform from tool coordinate frame to HL2 world frame, aka, the pose of the
            // tool in the virtual world
            PROFILE_BEGIN(ComputingPoseTransform);
            tool.PoseMatrix_HoloWorld = CorrespondenceMatcher::ComputeRigidTransform(
                tool.GeometryPoints,
                tool.ObservedPoints_World);
            PROFILE_END();

            tool.VisibleToHoloLens = true; // hooray

            /// Section:Remove points associated with a found tool to reduce our search size 
            /// in the next loop

            // sorts smallest to largest
            std::sort(indexList.begin(), indexList.end());

            // iterate from right to left to delete largest indices first
            for (auto it = indexList.crbegin(); it != indexList.crend(); ++it)
            {
                const int idx = *(it);
                validBlobData.erase(validBlobData.begin() + idx);
                collectedPoints.erase(collectedPoints.begin() + idx);
                blobPixelLocations.erase(blobPixelLocations.begin() + idx);
                collectedDepthPoints.erase(collectedDepthPoints.begin() + idx);
            }
        }
    }
    
    // could be deprecated in final version
    void SetToolListFromString(const std::string& encoded_string, std::map<uint8_t, IRTrackerUtils::TrackedTool>& toolDictionary)
    {
        // Lambda for splitting up a string with a delim
        auto splitString = [](const std::string& str, const std::string& delims)
        {
            std::vector<std::string> output;
            auto first = std::cbegin(str);

            while (first != std::cend(str))
            {
                const auto second = std::find_first_of(first, std::cend(str),
                    std::cbegin(delims), std::cend(delims));

                if (first != second) output.emplace_back(first, second);
                if (second == std::cend(str)) break;
                first = std::next(second);
            }

            return output;
        };

        auto delimitedMsg = splitString(encoded_string, ";");

        for (std::string toolSubstring : delimitedMsg)
        {
            if (toolSubstring.size() == 0) continue;
            IRTrackerUtils::TrackedTool tool;

            auto toolTripletsStr = splitString(toolSubstring, ",");

            //  this type technically means we can 'only' track 256 unique tools 
            //  (maybe don't try to stress-test this though...)
            tool.ID = static_cast<uint8_t>(std::stoi(toolTripletsStr[0]));

            for (size_t i = 0; i < (toolTripletsStr.size() - 1) / 3; i++)
            {
                tool.GeometryPoints.emplace_back(Eigen::Vector3d(
                    std::stod(toolTripletsStr[i * 3 + 1]),    // x coordinate 
                    std::stod(toolTripletsStr[i * 3 + 2]),    // y coordinate
                    std::stod(toolTripletsStr[i * 3 + 3])));  // z coordinate
            }

            // try_emplace should only keep the first instance if we add
            // a duplicate key
            toolDictionary.try_emplace(tool.ID, tool);
        }
    }

    //! @brief  Dump \param toolDictionary into an encoded double array. Each tool contains 18 elements, [id, visibleBit, 16 matrix elements]
    //! @param toolDictionary           Tool dictionary to serialize 
    //! @param out_encodedDoubleArray   Formatted double array to be processed elsewhere 
    void SerializeToolDictionary(const std::map<uint8_t, IRTrackerUtils::TrackedTool>& toolDictionary, std::vector<double>& out_encodedDoubleArray)
    {
        using namespace Eigen;
        out_encodedDoubleArray.clear();

        // 18 elements per tool : [id, visibleBit, 16 matrix elements]
        size_t estimatedSize = toolDictionary.size() * 18; 
        out_encodedDoubleArray.reserve(estimatedSize);

        for (const auto& [_, tool] : toolDictionary)
        {
            out_encodedDoubleArray.push_back(tool.ID);
            out_encodedDoubleArray.push_back(tool.VisibleToHoloLens ? 1 : 0);

            // stores in column major by default for Eigen
            const auto& poseMatrixData = tool.PoseMatrix_HoloWorld.data();
            out_encodedDoubleArray.insert(out_encodedDoubleArray.end(), poseMatrixData, poseMatrixData + 16);
        }
    }
}

Holo2IRTracker::Holo2IRTracker()
{
    // initialise caches
	m_cache_frameBlobInfo.reserve(100);
	m_cache_frameBlobPixelLocations.reserve(100);

    // assign space for these 'cache' cv::Mats
    m_ABImg16bit = cv::Mat(IMG_HEIGHT,IMG_WIDTH, CV_16UC1);
    m_DepthImg16bit = cv::Mat(IMG_HEIGHT,IMG_WIDTH, CV_16UC1);
    
    m_ABImg8bit = cv::Mat(IMG_HEIGHT,IMG_WIDTH, CV_8UC1);

    m_DepthDisplayImg8bit = cv::Mat(IMG_HEIGHT,IMG_WIDTH, CV_8UC1);
    m_ABDisplayImg8bit = cv::Mat(IMG_HEIGHT,IMG_WIDTH, CV_8UC1);
}

Holo2IRTracker::Holo2IRTracker(const std::string& encodedString, bool JSONString) : Holo2IRTracker()
{
    if (!JSONString) SetToolListFromString(encodedString, m_ToolDictionary);
    else IRTrackerUtils::JSONUtils::FillToolDictionaryFromJSONString(encodedString, m_ToolDictionary);
}

void Holo2IRTracker::ProcessLatestFrames(const uint16_t* ABImg, const uint16_t* DepthImg, 
    const Eigen::Ref<Eigen::Matrix4d> depth2world, const bool& UpdateDisplayImages)
{
    // main control loop function of this class
    using namespace IRTrackerUtils::ImageProc;

    // 1) Clear caches
    m_cache_frameBlobInfo.clear();
    m_cache_frameBlobPixelLocations.clear();

    // 2) Convert our sensor images to cv::Mats
    PROFILE_BEGIN(CVMatCreation);
    NativeToCVMat(ABImg, m_ABImg16bit, IMG_HEIGHT, IMG_WIDTH);
    NativeToCVMat(DepthImg, m_DepthImg16bit, IMG_HEIGHT, IMG_WIDTH);
    PROFILE_END();

    // 3) Process the IR image to brighten it so we can detect contours
    RebalanceImgAnd8Bit(m_ABImg16bit, m_ABImg8bit);
    
    if (UpdateDisplayImages) { 
        // clone it before blob detection does any more processing on m_ABImg8bit
        std::copy(m_ABImg8bit.datastart, m_ABImg8bit.dataend, m_ABDisplayImg8bit.data);        
    }

    BlobDetectionMethod method;
    if constexpr (USE_REFINED_BLOB_DETECT) { method = BlobDetectionMethod::RefineByScaling; }
    else method = BlobDetectionMethod::Basic;

    // 4) Find some circular looking blobs in 2D
    DetectBlobs2D(m_ABImg8bit, method, m_cache_frameBlobPixelLocations);
    
    // 5) Check if these circular blobs have meaningful depth locations and thus if they're 'valid' or not
    ValidateBlobs3D(m_DepthImg16bit, depth2world, m_cache_frameBlobPixelLocations, m_MapImageToUnitPlane, m_cache_frameBlobInfo);

    // 6) Examine all the valid 3D blobs in this frame, and then check if they correspond to tools we're tracking
    TryUpdatingToolDictionary(m_cache_frameBlobInfo, m_ToolDictionary);

    // 7) Optionally label and store our images for display elsewhere
    if (UpdateDisplayImages)
    {
        // add crosses at tool marker centres on the IR response image 
        LabelImageWithToolDictData(m_ToolDictionary, m_ABDisplayImg8bit);

        // process the depth image to produce an 8bit depth display texture
        GetProcessed8BitDepthImg(m_DepthImg16bit, m_DepthDisplayImg8bit);
    }
}

void Holo2IRTracker::ProcessLatestFrames(const uint16_t* ABImg, const uint16_t* DepthImg, const Eigen::Ref<Eigen::Matrix4d> depth2world)
{
	using namespace IRTrackerUtils::ImageProc;

    m_cache_frameBlobInfo.clear();
    m_cache_frameBlobPixelLocations.clear();

    NativeToCVMat(ABImg, m_ABImg16bit, IMG_HEIGHT, IMG_WIDTH);
    NativeToCVMat(DepthImg, m_DepthImg16bit, IMG_HEIGHT, IMG_WIDTH);

    RebalanceImgAnd8Bit(m_ABImg16bit, m_ABImg8bit);

    BlobDetectionMethod method;
    if (USE_REFINED_BLOB_DETECT) method = BlobDetectionMethod::RefineByScaling;
    else method = BlobDetectionMethod::Basic;
    
    DetectBlobs2D(m_ABImg8bit, method, m_cache_frameBlobPixelLocations);
    ValidateBlobs3D(m_DepthImg16bit, depth2world, m_cache_frameBlobPixelLocations, m_MapImageToUnitPlane, m_cache_frameBlobInfo);

    TryUpdatingToolDictionary(m_cache_frameBlobInfo, m_ToolDictionary);
}

int Holo2IRTracker::TrackedToolsCount()
{
	return m_ToolDictionary.size();
}

void Holo2IRTracker::GetSerializedToolDict(std::vector<double>& out_encodedDoubleArray)
{
    SerializeToolDictionary(m_ToolDictionary, out_encodedDoubleArray);
}

void Holo2IRTracker::RetrieveDisplayImages(cv::Mat& abImage8bit, cv::Mat& depthImage8bit)
{
    // copy the infrared response image
    if (m_ABDisplayImg8bit.size().area() == abImage8bit.size().area())
    {
        std::copy(m_ABDisplayImg8bit.datastart, m_ABDisplayImg8bit.dataend, abImage8bit.data);
    }

    // copy the depth map image
    if (m_DepthDisplayImg8bit.size().area() == depthImage8bit.size().area())
    {
        std::copy(m_DepthDisplayImg8bit.datastart, m_DepthDisplayImg8bit.dataend, depthImage8bit.data);
    }
}

void Holo2IRTracker::RetrieveDisplayImages(uint8_t* abImage8bit, uint8_t* depthImage8bit, size_t img_BufLen)
{
    if (img_BufLen != m_ABDisplayImg8bit.size().area() || img_BufLen != m_DepthDisplayImg8bit.size().area())
    {
        return;
    }

    if (!m_ABDisplayImg8bit.empty())
    {
        std::copy(m_ABDisplayImg8bit.datastart, m_ABDisplayImg8bit.dataend, abImage8bit);
    }

    if (!m_DepthDisplayImg8bit.empty())
    {
        std::copy(m_DepthDisplayImg8bit.datastart, m_DepthDisplayImg8bit.dataend, depthImage8bit);
    }
}

void Holo2IRTracker::SetUnmapFunction(IRTrackerUtils::UnmapFunction& unmapFunction)
{
    // should be attached to the depth sensor's unmap function
	m_MapImageToUnitPlane = unmapFunction;
}
