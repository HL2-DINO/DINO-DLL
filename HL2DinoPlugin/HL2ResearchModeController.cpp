#include "pch.h"
#include "HL2ResearchModeController.h"
#include "HL2ResearchModeController.g.cpp"
#include <DirectXMath.h>
#include "Holo2IRTracker.h"
#include "IRTrackerUtils.h"
#include "Shiny.h"

extern "C"
HMODULE LoadLibraryA(
    LPCSTR lpLibFileName
);

static ResearchModeSensorConsent camAccessCheck;
static HANDLE camConsentGiven;

typedef std::chrono::duration<int64_t, std::ratio<1, 10'000'000>> HundredsOfNanoseconds;
typedef std::chrono::milliseconds ms;
typedef std::chrono::microseconds micro_s;
typedef std::chrono::duration<float> fsec;

constexpr bool ENABLE_PROFILER = true;

//! @name Anonymous functions
//!@{
long long checkAndConvertUnsigned(UINT64 val)
{
    return static_cast<long long>(val);
}

//! @brief Used to convert from DirectX style matrices to Eigen (transposes the input from row-major to output column-major)
//! @param in       A row-major DirectX XMMATRIX (float)
//! @param out      A column-major (standard) Eigen Matrix4d (double)
void DX2EigenConvert(const DirectX::XMMATRIX& in, Eigen::Ref<Eigen::Matrix4d> out)
{
    // Direct quote from: https://learn.microsoft.com/en-us/windows/win32/dxmath/pg-xnamath-getting-started
    // 'DirectXMath uses row-major matrices, row vectors, and pre-multiplication'

    // Eigen is column major: and will use post-multiplication like most other libraries
    // so this function will also transpose the DirectX matrix, as we will keep Eigen's convention

    // load into more easily accessible class
    DirectX::XMFLOAT4X4 dxMatVals; XMStoreFloat4x4(&dxMatVals, in);

    // Define a lambda for the static_cast operation
    auto toDouble = [](float value) -> double { return static_cast<double>(value); };

    // Note: the '<<' stream operator assumes you're loading the matrix in row-major order in Eigen
    // and the dxMat float value index '_12' corresponds to row 1 and column 2
    out << toDouble(dxMatVals._11), toDouble(dxMatVals._21), toDouble(dxMatVals._31), toDouble(dxMatVals._41),
        toDouble(dxMatVals._12), toDouble(dxMatVals._22), toDouble(dxMatVals._32), toDouble(dxMatVals._42),
        toDouble(dxMatVals._13), toDouble(dxMatVals._23), toDouble(dxMatVals._33), toDouble(dxMatVals._43),
        toDouble(dxMatVals._14), toDouble(dxMatVals._24), toDouble(dxMatVals._34), toDouble(dxMatVals._44);

    // Strong suggestion: do not try and change Eigen or DirectX convention in this project 
    // Possible TODO: Can we verify at runtime that the DirectX Matrix passed in here is definitely row major? 
}
//!@}

namespace winrt::HL2DinoPlugin::implementation
{
    HL2ResearchModeController::HL2ResearchModeController()
    {
        // Load Research Mode library
        camConsentGiven = CreateEvent(nullptr, true, false, nullptr);
        HMODULE hrResearchMode = LoadLibraryA("ResearchModeAPI");
        HRESULT hr = S_OK;

        if (hrResearchMode)
        {
            typedef HRESULT(__cdecl* PFN_CREATEPROVIDER) (IResearchModeSensorDevice** ppSensorDevice);
            PFN_CREATEPROVIDER pfnCreate = reinterpret_cast<PFN_CREATEPROVIDER>(GetProcAddress(hrResearchMode, "CreateResearchModeSensorDevice"));
            if (pfnCreate)
            {
                winrt::check_hresult(pfnCreate(&m_pSensorDevice));
            }
            else
            {
                winrt::check_hresult(E_INVALIDARG);
            }
        }

        using namespace winrt::Windows::Perception::Spatial::Preview;
        // get spatial locator of rigNode
        GUID guid;
        IResearchModeSensorDevicePerception* pSensorDevicePerception;
        winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception)));
        winrt::check_hresult(pSensorDevicePerception->GetRigNodeId(&guid));
        pSensorDevicePerception->Release();

        // for world frame calcs later
        m_locator = SpatialGraphInteropPreview::CreateLocatorForNode(guid);

        // sensor consent checks
        winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&m_pSensorDeviceConsent)));
        winrt::check_hresult(m_pSensorDeviceConsent->RequestCamAccessAsync(HL2ResearchModeController::CamAccessOnComplete));

        // This call makes cameras run at full frame rate. Normally they are optimized
        // for headtracker use. For some applications that may be sufficient
        m_pSensorDevice->DisableEyeSelection();

        // Dump the tags/info about the headset sensor to member var
        size_t sensorCount = 0;
        winrt::check_hresult(m_pSensorDevice->GetSensorCount(&sensorCount));
        m_sensorDescriptors.resize(sensorCount);
        winrt::check_hresult(m_pSensorDevice->GetSensorDescriptors(m_sensorDescriptors.data(), m_sensorDescriptors.size(), &sensorCount));

        // set the profiler on/off
        PROFILE_SET_ENABLED(ENABLE_PROFILER);
    }

    HL2ResearchModeController::HL2ResearchModeController(hstring const& toolConfigString, bool isJsonFormattedString) : HL2ResearchModeController()
    {
        // appropriately initialise the IR tracking class
        m_IRTracker = Holo2IRTracker::Holo2IRTracker(winrt::to_string(toolConfigString), isJsonFormattedString);

        // tool vector size is 18 [16 tool pose matrices, and 2 informational 'bits'], and reserving x2 this size
        // as a safety measure
        m_OutputToolPoseVector.reserve(static_cast<size_t>(m_IRTracker.TrackedToolsCount()) * 18 * 2);
    }

    //! @brief  Checks the internally declared \p camConsentGiven variable to validate for user permission
   //!         of sensor access consent. 
   //! 
   //! @note   Implementation from petergu684 GitHub project HoloLens2-ResearchMode-Unity
    HRESULT CheckCamConsent()
    {
        HRESULT hr = S_OK;
        DWORD waitResult = WaitForSingleObject(camConsentGiven, INFINITE);
        if (waitResult == WAIT_OBJECT_0)
        {
            switch (camAccessCheck)
            {
            case ResearchModeSensorConsent::Allowed:
                OutputDebugString(L"Access is granted");
                break;
            case ResearchModeSensorConsent::DeniedBySystem:
                OutputDebugString(L"Access is denied by the system");
                hr = E_ACCESSDENIED;
                break;
            case ResearchModeSensorConsent::DeniedByUser:
                OutputDebugString(L"Access is denied by the user");
                hr = E_ACCESSDENIED;
                break;
            case ResearchModeSensorConsent::NotDeclaredByApp:
                OutputDebugString(L"Capability is not declared in the app manifest");
                hr = E_ACCESSDENIED;
                break;
            case ResearchModeSensorConsent::UserPromptRequired:
                OutputDebugString(L"Capability user prompt required");
                hr = E_ACCESSDENIED;
                break;
            default:
                OutputDebugString(L"Access is denied by the system");
                hr = E_ACCESSDENIED;
                break;
            }
        }
        else
        {
            hr = E_UNEXPECTED;
        }
        return hr;
    }

    void HL2ResearchModeController::CamAccessOnComplete(ResearchModeSensorConsent consent)
    {
        camAccessCheck = consent;
        SetEvent(camConsentGiven);
    }

    void HL2ResearchModeController::InitialiseDepthSensor()
    {
        using namespace DirectX;
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == ResearchModeSensorType::DEPTH_AHAT)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_depthSensor));
                winrt::check_hresult(m_depthSensor->QueryInterface(IID_PPV_ARGS(&m_pDepthCameraSensor)));

                // matrix to convert from rig/LF coordinates to depth coordinates
                winrt::check_hresult(m_pDepthCameraSensor->GetCameraExtrinsicsMatrix(&m_depthCamExtrinsic));

                // matrix to convert from depth to rig/LF coordinates
                m_depthCamExtrinsicInverse = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_depthCamExtrinsic));
                break;
            }
        }
    }
    void HL2ResearchModeController::StartDepthSensorLoop()
    {
        // check if a coordinate frame has been set
        if (m_refFrame == nullptr) m_refFrame = m_locator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem();

        //m_pDepthUpdateThread = new std::thread(HL2ResearchMode::DepthSensorLoop, this);

        if (SUCCEEDED(CheckCamConsent())) {
            m_pDepthUpdateThread = new std::thread(HL2ResearchModeController::DepthSensorLoop, this);
        }
    }
    void HL2ResearchModeController::StopSensorLoop()
    {
        m_depthSensorLoopStarted = false;

        if (m_RawDepthImgBuf)
        {
            delete[] m_RawDepthImgBuf;
            m_RawDepthImgBuf = nullptr;
        }
        if (m_8bitDepthImgBuf)
        {
            delete[] m_8bitDepthImgBuf;
            m_8bitDepthImgBuf = nullptr;
        }

        if (m_RawABImgBuf) { delete[] m_RawABImgBuf; m_RawABImgBuf = nullptr; }
        if (m_8BitABImgBuf) { delete[] m_8BitABImgBuf; m_8BitABImgBuf = nullptr; }

        if (m_pSensorDevice) { m_pSensorDevice->Release(); m_pSensorDevice = nullptr; }
        if (m_pSensorDeviceConsent) { m_pSensorDeviceConsent->Release(); m_pSensorDeviceConsent = nullptr; }
    }

    void HL2ResearchModeController::SetReferenceCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& coordinateFrame)
    {
        m_refFrame = coordinateFrame;
    }

    void HL2ResearchModeController::SetToolListByString(hstring const& toolListString)
    {
        m_IRTracker = Holo2IRTracker::Holo2IRTracker(winrt::to_string(toolListString));

        m_OutputToolPoseVector.reserve(static_cast<size_t>(m_IRTracker.TrackedToolsCount()) * 18 * 2);
    }

    void HL2ResearchModeController::ToggleDisplaySensorImages(bool showTextures)
    {
        std::lock_guard<std::mutex> lock(m_toggleImgMutex);
        m_stashSensorImgs = showTextures;
    }

    bool HL2ResearchModeController::ToolDictionaryUpdated()
    {
        std::lock_guard<std::mutex> l(m_toolDoubleVectorMutex);
        return m_toolDictUpdated.load();
    }

    bool HL2ResearchModeController::RawDepthImageUpdated()
    {
        return m_RawDepthImageUpdated.load(std::memory_order_relaxed);
    }
    bool HL2ResearchModeController::RawABImageUpdated()
    {
        return m_RawABImageUpdated.load(std::memory_order_relaxed);
    }
    bool HL2ResearchModeController::Depth8BitImageUpdated()
    {
        return m_Depth8BitImageUpdated.load(std::memory_order_relaxed);
    }
    bool HL2ResearchModeController::AB8BitImageUpdated()
    {
        return m_AB8BitImageUpdated.load(std::memory_order_relaxed);
    }

    com_array<double> HL2ResearchModeController::GetTrackedToolsPoseMatrices()
    {
        std::lock_guard<std::mutex> l(m_imgMutex);
        com_array<double> tools2worldArr = com_array<double>(m_OutputToolPoseVector.begin(), m_OutputToolPoseVector.end());
        m_toolDictUpdated.store(false, std::memory_order_relaxed); // flag is reset
        return tools2worldArr;
    }

    com_array<uint16_t> HL2ResearchModeController::GetRawDepthImageBuffer()
    {
        std::lock_guard<std::mutex> l(m_imgMutex);
        if (!m_RawDepthImgBuf) return com_array<uint16_t>();
        com_array<UINT16> tempBuffer = com_array<UINT16>(m_RawDepthImgBuf, m_RawDepthImgBuf + m_depthBufferSize);

        return tempBuffer;
    }

    com_array<uint16_t> HL2ResearchModeController::GetRawABImageBuffer()
    {
        std::lock_guard<std::mutex> l(m_imgMutex);
        if (!m_RawABImgBuf) return com_array<uint16_t>();

        com_array<UINT16> tempBuffer = com_array<UINT16>(m_RawABImgBuf, m_RawABImgBuf + m_depthBufferSize);

        return tempBuffer;
    }

    com_array<uint8_t> HL2ResearchModeController::Get8BitDepthImageBuf()
    {
        std::lock_guard<std::mutex> l(m_imgMutex);
        if (!m_8bitDepthImgBuf) return com_array<UINT8>();
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_8bitDepthImgBuf), std::move_iterator(m_8bitDepthImgBuf + m_depthBufferSize));

        m_Depth8BitImageUpdated.store(false, std::memory_order_relaxed);
        return tempBuffer;
    }

    com_array<uint8_t> HL2ResearchModeController::Get8BitABImageBuf()
    {
        std::lock_guard<std::mutex> l(m_imgMutex);
        if (!m_8BitABImgBuf) return com_array<UINT8>();

        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_8BitABImgBuf), std::move_iterator(m_8BitABImgBuf + m_depthBufferSize));

        m_AB8BitImageUpdated.store(false, std::memory_order_relaxed);
        return tempBuffer;
    }

    hstring HL2ResearchModeController::GetProfilerString()
    {
        PROFILE_UPDATE();
        std::string tempString = PROFILE_GET_TREE_STRING();
        PROFILE_CLEAR(); // profile data cleared before next call
        return winrt::to_hstring(tempString);
    }

    void HL2ResearchModeController::DepthSensorLoop(HL2ResearchModeController* pHL2ResearchMode)
    {
        using namespace winrt::Windows::Perception;
        using namespace DirectX;

        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_depthSensorLoopStarted) pHL2ResearchMode->m_depthSensorLoopStarted = true;
        else return;

        pHL2ResearchMode->m_depthSensor->OpenStream();

        // Capture the unmap function as a std::function and then give it to our IR tracking class
        IRTrackerUtils::UnmapFunction unmapLambda =
            [&pDepthSensor = pHL2ResearchMode->m_pDepthCameraSensor] // object to reference
        (float(&uv)[2], float(&xy)[2]) // function inputs
        {
            if (!pDepthSensor) return false; // means the sensor ptr is null

            // check the hresult
            if SUCCEEDED(pDepthSensor->MapImagePointToCameraUnitPlane(uv, xy))
            {
                return true; // should have updated the values of 'xy'
            }

            else return false;
        };

        pHL2ResearchMode->m_IRTracker.SetUnmapFunction(unmapLambda);

        ResearchModeSensorTimestamp lastTimestamp = ResearchModeSensorTimestamp();
        lastTimestamp.HostTicks = 0;

        try
        {
            while (pHL2ResearchMode->m_depthSensorLoopStarted)
            {
                PROFILE_BLOCK(OneFullLoop);

                IResearchModeSensorFrame* pDepthSensorFrame = nullptr;
                ResearchModeSensorResolution resolution;

                PROFILE_BEGIN(ResearchModeGetNextBuffer);
                pHL2ResearchMode->m_depthSensor->GetNextBuffer(&pDepthSensorFrame);
                PROFILE_END();

                {
                    ResearchModeSensorTimestamp timestamp;
                    pDepthSensorFrame->GetTimeStamp(&timestamp);

                    // on-board HL2 clock? could use sensor clock
                    if (timestamp.HostTicks == lastTimestamp.HostTicks)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                        continue; // wait for new frame to arrive
                    }

                    //  If we reach here, it means the timestamps are different, 
                    //  so update lastTimestamp
                    lastTimestamp = timestamp;
                }

                // process sensor frame
                pDepthSensorFrame->GetResolution(&resolution);
                pHL2ResearchMode->m_depthResolution = resolution;

                // refresh depth frame for active brightness + depth frames
                IResearchModeSensorDepthFrame* pDepthFrame = nullptr;
                winrt::check_hresult(pDepthSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame)));

                size_t outBufferCount = 0;
                const UINT16* pDepth = nullptr;
                pDepthFrame->GetBuffer(&pDepth, &outBufferCount);
                pHL2ResearchMode->m_depthBufferSize = outBufferCount;
                size_t outAbBufferCount = 0;
                const UINT16* pAbImage = nullptr;
                pDepthFrame->GetAbDepthBuffer(&pAbImage, &outAbBufferCount);

                // get tracking transform
                PROFILE_BEGIN(LocateWorld);
                ResearchModeSensorTimestamp timestamp;
                pDepthSensorFrame->GetTimeStamp(&timestamp);

                auto ts = PerceptionTimestampHelper::FromSystemRelativeTargetTime(
                    HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp.HostTicks))
                );

                // calculate the pose of headset rig coordinate system w.r.t world frame
                auto transToWorld = pHL2ResearchMode->m_locator.TryLocateAtTimestamp(ts, pHL2ResearchMode->m_refFrame);
                // headset couldn't be located in world frame
                if (transToWorld == nullptr) { continue; }

                auto rot = transToWorld.Orientation();
                auto quatInDx = XMFLOAT4(rot.x, rot.y, rot.z, rot.w);
                auto rotMat = XMMatrixRotationQuaternion(XMLoadFloat4(&quatInDx));
                auto pos = transToWorld.Position();
                auto posMat = XMMatrixTranslation(pos.x, pos.y, pos.z);

                // if headset-rig pose is known, multiply by the inverse extrinsic transform between the depthCamera and 
                // LF sensor. calculates the depthToWorld matrix (pre-multiply by DirectX convention)
                // Goes from [depthCam -> LF Cam -> World Frame]
                auto depthToWorld = pHL2ResearchMode->m_depthCamExtrinsicInverse * rotMat * posMat;

                Eigen::Matrix4d eig_depthToWorld, eig_world2depth;

                // convert from DirectX to Eigen
                DX2EigenConvert(depthToWorld, eig_depthToWorld);
                eig_world2depth = eig_depthToWorld.inverse().eval();
                PROFILE_END();

                bool StashTexThisFrame = true;
                pHL2ResearchMode->m_toggleImgMutex.lock();
                StashTexThisFrame = pHL2ResearchMode->m_stashSensorImgs;
                pHL2ResearchMode->m_toggleImgMutex.unlock();

                // Main logic:
                // pass in the newest AB frame, depth Frame, and current pose matrix to the IR Tracking class
                // internally, this will update its Tool Dictionary/Map structure
                PROFILE_BEGIN(ImgProcessingPipeline);
                pHL2ResearchMode->m_IRTracker.ProcessLatestFrames(pAbImage, pDepth, eig_depthToWorld, StashTexThisFrame);
                PROFILE_END();

                // Serialize the values in the tool dictionary and output it into the double array
                // of this class, which can be accessed by Unity externally. Everything is in right-handed
                // coordinates and metres in this result
                pHL2ResearchMode->m_toolDoubleVectorMutex.lock();
                pHL2ResearchMode->m_IRTracker.GetSerializedToolDict(pHL2ResearchMode->m_OutputToolPoseVector);
                pHL2ResearchMode->m_toolDictUpdated.store(true, std::memory_order_relaxed);
                pHL2ResearchMode->m_toolDoubleVectorMutex.unlock();

                if (StashTexThisFrame) // set this to false externally to cut out img stashing operations 
                {
                    PROFILE_BLOCK(SavingSensorImages);
                    // Raw textures to store ab + depth frames as images
                    auto pDepthTexture = std::make_unique<uint8_t[]>(outBufferCount);
                    auto pAbTexture = std::make_unique<uint8_t[]>(outAbBufferCount);

                    pHL2ResearchMode->m_IRTracker.RetrieveDisplayImages(pAbTexture.get(), pDepthTexture.get(), 512 * 512);

                    // ------------------------------------------------

                    // save data
                    {
                        std::lock_guard<std::mutex> l(pHL2ResearchMode->m_imgMutex);

                        // save raw depth map
                        if (!pHL2ResearchMode->m_RawDepthImgBuf) { pHL2ResearchMode->m_RawDepthImgBuf = new UINT16[outBufferCount]; }
                        memcpy(pHL2ResearchMode->m_RawDepthImgBuf, pDepth, outBufferCount * sizeof(UINT16));

                        // save pre-processed depth map texture (for visualization)
                        if (!pHL2ResearchMode->m_8bitDepthImgBuf) { pHL2ResearchMode->m_8bitDepthImgBuf = new UINT8[outBufferCount]; }
                        memcpy(pHL2ResearchMode->m_8bitDepthImgBuf, pDepthTexture.get(), outBufferCount * sizeof(UINT8));

                        // save raw AbImage
                        if (!pHL2ResearchMode->m_RawABImgBuf) { pHL2ResearchMode->m_RawABImgBuf = new UINT16[outBufferCount]; }
                        memcpy(pHL2ResearchMode->m_RawABImgBuf, pAbImage, outBufferCount * sizeof(UINT16));

                        // save pre-processed AbImage texture (for visualization)
                        if (!pHL2ResearchMode->m_8BitABImgBuf) { pHL2ResearchMode->m_8BitABImgBuf = new UINT8[outBufferCount]; }
                        memcpy(pHL2ResearchMode->m_8BitABImgBuf, pAbTexture.get(), outBufferCount * sizeof(UINT8));

                        pHL2ResearchMode->m_AB8BitImageUpdated.store(true, std::memory_order_relaxed);
                        pHL2ResearchMode->m_Depth8BitImageUpdated.store(true, std::memory_order_relaxed);
                    }

                    pDepthTexture.reset();
                }

                // release space
                if (pDepthFrame) pDepthFrame->Release();
                if (pDepthSensorFrame) pDepthSensorFrame->Release();
            }
        }
        catch (...) {}
        pHL2ResearchMode->m_depthSensor->CloseStream();
        pHL2ResearchMode->m_depthSensor->Release();
        pHL2ResearchMode->m_depthSensor = nullptr;
    }

    std::string HL2ResearchModeController::MatrixToString(DirectX::XMFLOAT4X4 mat)
    {
        std::stringstream ss;
        ss << "XMat:\t";
        for (size_t i = 0; i < 4; i++)
        {
            for (size_t j = 0; j < 4; j++)
            {
                ss << mat(i, j) << ",";
            }
            ss << "\n";
        }
        return ss.str();
    }

    HL2ResearchModeController::~HL2ResearchModeController()
    {
        StopSensorLoop();
    }
}
