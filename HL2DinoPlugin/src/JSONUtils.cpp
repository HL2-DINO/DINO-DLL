#include "pch.h"
#include <winrt/Windows.Data.Json.h>
#include "IRTrackerUtils.h"
#include <sstream>

/**
 * @file        JSONUtils.cpp
 * @brief       Utils for handling JSON config files for the DINO-DLL app
 * @author      Hisham Iqbal
 * @copyright   &copy; Hisham Iqbal 2023
 *
 */

namespace
{
    using namespace winrt::Windows::Data::Json;
    typedef std::vector<Eigen::Vector3d> PointsXYZList;

    static bool ParseAndFillCoordinatesList(const JsonObject& toolObject, PointsXYZList& coordinatesVector)
    {
        using namespace winrt::Windows::Data::Json;
        coordinatesVector.clear();
        JsonArray toolCoordinateSet = toolObject.GetNamedArray(L"coordinates", nullptr);
        if (!toolCoordinateSet) return false;

        for (auto const toolTriplet : toolCoordinateSet)
        {
            if (toolTriplet.ValueType() != JsonValueType::Array) return false;

            JsonArray coordinateArray = toolTriplet.GetArray();
            if (coordinateArray.Size() != 3) return false;

            auto xVal = coordinateArray.GetAt(0);
            auto yVal = coordinateArray.GetAt(1);
            auto zVal = coordinateArray.GetAt(2);

            if (xVal.ValueType() != JsonValueType::String ||
                yVal.ValueType() != JsonValueType::String ||
                zVal.ValueType() != JsonValueType::String) return false;

            try
            {
                double x = std::stod(xVal.GetString().c_str());
                double y = std::stod(yVal.GetString().c_str());
                double z = std::stod(zVal.GetString().c_str());
                coordinatesVector.emplace_back(Eigen::Vector3d(x, y, z));
            }
            catch (...)
            {
                return false;
            }
        }

        return true;
    }

    int GetId(const JsonObject& toolObject)
    {
        using namespace winrt::Windows::Data::Json;
        IJsonValue idVal = toolObject.GetNamedValue(L"id", nullptr);
        return (idVal.ValueType() == JsonValueType::Number) ? static_cast<uint8_t>(idVal.GetNumber()) : -1;
    }
}

namespace IRTrackerUtils::JSONUtils
{
	void FillToolDictionaryFromJSONString(const std::string jsonString, std::map<uint8_t, TrackedTool>& toolDictionary)
	{
        using namespace winrt::Windows::Data::Json;
        JsonObject toolJsonObject;
        // failure to read a valid JSON string returns early
        if(!JsonObject::TryParse(winrt::to_hstring(jsonString), toolJsonObject)) return; 
        
        // failure to find a 'tools' array also returns early
        JsonArray toolArray = toolJsonObject.GetNamedArray(L"tools", nullptr);
        if (toolArray == nullptr) return;

        int idx;
        PointsXYZList toolCoordinateSet;
        for (auto const toolValue : toolArray)
        {
            auto toolObject = toolValue.GetObject();

            idx = GetId(toolObject);
            if (idx < 0) continue;
            if (!ParseAndFillCoordinatesList(toolObject, toolCoordinateSet)) continue;

            TrackedTool emptyTool;
            emptyTool.ID = static_cast<uint8_t>(idx);
            emptyTool.GeometryPoints = toolCoordinateSet;
            toolDictionary.try_emplace(idx, emptyTool);
        }
	}
}