#include "pch.h"
#include "IRTrackerUtils.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "Shiny.h"

/**
 * @file        IRImageProcUtils.cpp
 * @brief       Utility image processing functions
 * @author      Hisham Iqbal
 * @copyright   &copy; Hisham Iqbal 2023
 *
 */


namespace // Anonymous ImageProc
{
    static constexpr uint8_t BINARY_THRESH_8BIT = 180;
    static constexpr uint16_t THRESH_RAW_DEPTH_16BIT = 4090;
    static constexpr double PI = 3.141592653589793238462;
    
    void DetectBlobs2DBasic(cv::Mat& processed_image, std::vector<cv::Point2f>& outPixelLocations)
    {
        PROFILE_BLOCK(DetectBlobsBasic);
        if (outPixelLocations.size() > 0) outPixelLocations.clear();

        // binarisation to speed up contour detect
        cv::threshold(processed_image, processed_image, BINARY_THRESH_8BIT, 255, cv::THRESH_BINARY);

        // external contours of interest in this scenario
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(processed_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double area, perimeter, circ;
        float blobX, blobY;

        for (const std::vector<cv::Point> contour : contours)
        {
            area = cv::contourArea(contour);
            perimeter = cv::arcLength(contour, true);

            /* filter by area. max: 1/16th of total image   */
            if (area < 5 || area > 16384) continue;

            /* see https://en.wikipedia.org/wiki/Roundness  */
            /* for definition of roundedness formula used   */
            circ = (4 * PI * area) / (perimeter * perimeter);
            if (circ < 0.7f) continue; // filter by circularity

            // see https://en.wikipedia.org/wiki/Image_moment 
            // for defining a centroid from an image moment
            auto M = cv::moments(contour);
            blobX = M.m10 / M.m00;
            blobY = M.m01 / M.m00;
            outPixelLocations.emplace_back(blobX, blobY);
        }
    }

    void DetectBlobs2DRefined(cv::Mat& processed_image, std::vector<cv::Point2f>& outPixelLocations)
    {
        PROFILE_BLOCK(DetectBlobsRefined);
        using namespace Eigen;
        if (outPixelLocations.size() > 0) outPixelLocations.clear();

        // binarisation for helping contour detection, floor all below BINARY_THRESH to 0, and ceil above to 255
        cv::threshold(processed_image, processed_image, BINARY_THRESH_8BIT, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(processed_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double area, perimeter, circ;
        cv::Size2i size = processed_image.size();

        for (const std::vector<cv::Point> contour : contours)
        {
            area = cv::contourArea(contour);
            perimeter = cv::arcLength(contour, true);

            // filter by area, dropped
            if (area < 5 || area > 16384) continue;

            auto boundRect = cv::boundingRect(contour);
            auto xmin = boundRect.x; auto xmax = xmin + boundRect.width;
            auto ymin = boundRect.y; auto ymax = ymin + boundRect.height;

            size_t bound_radius = 1;

            xmin = (xmin > bound_radius) ? xmin - bound_radius : 0;
            ymin = (ymin > bound_radius) ? ymin - bound_radius : 0;
            
            
            xmax = (xmax < size.width - bound_radius - 1) ? xmax + bound_radius : size.width - 1;
            ymax = (ymax < size.height - bound_radius - 1) ? ymax + bound_radius : size.height - 1;

            auto ROI = cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin);
            auto crop = processed_image(ROI);

            auto SFx = 200.0 / crop.size().width;
            auto SFy = 200.0 / crop.size().height;
            auto sf = std::min<double>(SFx, SFy);

            cv::resize(crop, crop, cv::Size(0, 0), sf, sf, cv::INTER_LINEAR);

            std::vector<std::vector<cv::Point>> large_contours;
            cv::threshold(crop, crop, BINARY_THRESH_8BIT, 255, cv::THRESH_BINARY);

            cv::findContours(crop, large_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            area = cv::contourArea(contour);
            perimeter = cv::arcLength(contour, true);

            if (large_contours.size() > 0)
            {
                auto new_contour = large_contours[0];

                /* see https://en.wikipedia.org/wiki/Roundness  */
                /* for definition of roundedness formula used   */
                area = cv::contourArea(new_contour);
                perimeter = cv::arcLength(new_contour, true);
                circ = (4 * PI * area) / (perimeter * perimeter);
                if (circ < 0.7f) continue; // filter by circularity

                auto Ellipse = cv::fitEllipseDirect(new_contour);

                float blobX = Ellipse.center.x; blobX /= sf; blobX += xmin;
                float blobY = Ellipse.center.y; blobY /= sf; blobY += ymin;

                outPixelLocations.emplace_back(blobX, blobY);
            }
        }
    }
}

namespace IRTrackerUtils
{
    float ImageProc::BilinearInterpolation(const cv::Mat& image, const cv::Point2f& point) 
    {
        if (image.empty()) return -1.0f;

        float x = point.x;
        float y = point.y;
        int x0 = static_cast<int>(x);
        int y0 = static_cast<int>(y);
       
        if (x0 < 0 || y0 < 0 || x >= image.cols || y >= image.rows) { return -1; }
        int x1 = std::min(x0 + 1, image.cols - 1);
        int y1 = std::min(y0 + 1, image.rows - 1);

        float dx = x - x0;
        float dy = y - y0;

        float q00(-1), q01(-1), q10(-1), q11(-1);

        if (image.type() == CV_8UC1)
        {
            q00 = static_cast<float>(image.at<uint8_t>(y0, x0));
            q01 = static_cast<float>(image.at<uint8_t>(y0, x1));
            q10 = static_cast<float>(image.at<uint8_t>(y1, x0));
            q11 = static_cast<float>(image.at<uint8_t>(y1, x1));
        }
        else if (image.type() == CV_16UC1)
        {
            q00 = static_cast<float>(image.at<uint16_t>(y0, x0));
            q01 = static_cast<float>(image.at<uint16_t>(y0, x1));
            q10 = static_cast<float>(image.at<uint16_t>(y1, x0));
            q11 = static_cast<float>(image.at<uint16_t>(y1, x1));
        }
        else return -1;
 
        // bilinear interp for the special case that our y1 - y0 is always 1 
        float result = (q00) * (1 - dx) * (1 - dy) +
                       (q01) * dx * (1 - dy) +
                       (q10) * (1 - dx) * dy +
                       (q11) * dx * dy;

        return result;
    }

	template<typename T>
	void ImageProc::NativeToCVMat(const T* src, cv::Mat& dst, int rows, int cols)
	{
        int chs = 1; // we only support grayscale for now
		// Create Mat from buffer 
		// assert that the types match
		if (CV_MAKE_TYPE(cv::DataType<T>::type, chs) != dst.type()) return; // return unchanged
		// assert that the total number of bytes matches?

		std::memcpy(dst.data, src, static_cast<unsigned long long>(rows) * cols * chs * sizeof(T));
	}
   
	void ImageProc::DetectBlobs2D(cv::Mat& processed_image, BlobDetectionMethod method, std::vector<cv::Point2f>& outPixelLocations)
	{
		switch (method) 
		{
		    case BlobDetectionMethod::Basic:
                DetectBlobs2DBasic(processed_image, outPixelLocations);
			    break;

		    case BlobDetectionMethod::RefineByScaling:
                DetectBlobs2DRefined(processed_image, outPixelLocations);
			    break;

		    default:
                DetectBlobs2DBasic(processed_image, outPixelLocations);
                break;
		}
	}

    void ImageProc::ValidateBlobs3D(const cv::Mat&                      inDepthImg, 
                                    const Eigen::Ref<Eigen::Matrix4d>   inDepth2World, 
                                    const std::vector<cv::Point2f>&     inBlobPixels2D, 
                                    const UnmapFunction                 MapImagePointToCameraUnitPlane, 
                                    std::vector<InfraBlobInfo>&         outBlobInfo)
    {
        PROFILE_BLOCK(ValidateBlobs3D);
        using namespace Eigen;
        if (outBlobInfo.size() > 0) outBlobInfo.clear();

        // if we can't access the depth-camera's unmap function, exit
        if (!MapImagePointToCameraUnitPlane) { return; }

        Vector3d pointInDepth, pointInWorld;
        Eigen::Affine3d transform(inDepth2World);
        for (const auto& pixelLocation : inBlobPixels2D)
        {
            const float depthVal = BilinearInterpolation(inDepthImg, pixelLocation);
            
            // check: https://github.com/microsoft/HoloLens2ForCV/blob/main/Samples/SensorVisualization/SensorVisualization/Content/SlateCameraRenderer.cpp
            // for the magic val of 4090 for depth AHAT
            if (depthVal == 0 || depthVal > 4090) { continue; }

            float xy[2] = { 0.0,0.0 };
            float uv[2] = { pixelLocation.x, pixelLocation.y };

            // unmap to unit plane, function should return false in case
            // of any 'bad' inputs
            if (!MapImagePointToCameraUnitPlane(uv, xy)) continue;
            
            pointInDepth = Vector3d(static_cast<double>(xy[0]), 
                                    static_cast<double>(xy[1]), 
                                                           1);

            pointInDepth.normalize(); // turn it into a unit vector
            pointInDepth *= (static_cast<double>(depthVal) / 1000.0); // convert into metres
            pointInWorld = transform * pointInDepth.homogeneous();

            InfraBlobInfo valid_blob{ cv::Point2f(pixelLocation.x, pixelLocation.y), pointInDepth, pointInWorld };
            outBlobInfo.emplace_back(valid_blob);
        }
    }

    void ImageProc::RebalanceImgAnd8Bit(cv::Mat& inputRaw16BitImg, cv::Mat& output8BitImg)
    {
        if (inputRaw16BitImg.type() != CV_16UC1) return; // routine is optimised for this
        if (output8BitImg.type() != CV_8UC1) output8BitImg = cv::Mat(inputRaw16BitImg.rows, inputRaw16BitImg.cols, CV_8UC1);
        PROFILE_BEGIN(ABImageProcessing);
        // first and last elements of image
        uint16_t* pStart = (uint16_t*)inputRaw16BitImg.data;
        const uint16_t* pEnd = pStart + (static_cast<size_t>(inputRaw16BitImg.rows) * inputRaw16BitImg.cols);

        for (uint16_t* ptr = pStart; ptr != pEnd; ++ptr) *(ptr) >>= 2;

        /// cv::convertTo will internally use OpenCV's saturateCast 
        /// function. So linear remapping needed from above to avoid
        /// accidentally maxing out loads of pixels

        inputRaw16BitImg.convertTo(output8BitImg, CV_8UC1);
        PROFILE_END();
    }

    void ImageProc::LabelImageWithToolDictData(const std::map<uint8_t, IRTrackerUtils::TrackedTool>& toolDictionary, cv::Mat Img2Label)
    {
        PROFILE_BLOCK(AnnotatingImages);
        for (const auto& [_, tool] : toolDictionary)
        {
            for (const auto& MarkerCentre : tool.ObservedImgKeypoints)
            {
                cv::drawMarker(Img2Label, MarkerCentre, cv::Scalar(100, 100, 100), cv::MARKER_CROSS, 25, 5);
            }           
        }
    }
    
    void ImageProc::GetProcessed8BitDepthImg(const cv::Mat& input16bitdepthImg, cv::Mat& output8bitDepth)
    {
        PROFILE_BLOCK(ProcessingDepthImg);
        cv::Mat processed16Bit = cv::Mat(input16bitdepthImg.rows, input16bitdepthImg.cols, CV_16UC1);
        // should set any pixels above 4090 to 0 and leave everything else untouched
        // magic val of 4090 comes from ResearchModeForCV 
        // https://github.com/microsoft/HoloLens2ForCV/blob/main/Samples/SensorVisualization/SensorVisualization/Content/SlateCameraRenderer.cpp
           
        cv::threshold(input16bitdepthImg, processed16Bit, THRESH_RAW_DEPTH_16BIT, 0, cv::THRESH_TOZERO_INV);
        processed16Bit.convertTo(output8bitDepth, CV_8UC1, 255.0f/1000); // so a depth value of 1m is max brightness
    }
}

