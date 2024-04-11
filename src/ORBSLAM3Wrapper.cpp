#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <ORB_SLAM3/include/KeyFrame.h>
#include <ORB_SLAM3/include/Converter.h>
#include <ORB_SLAM3/include/Tracking.h>
#include <ORB_SLAM3/include/MapPoint.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ORBSLAM3Wrapper.h"
#include "NDArrayConverter.h"

namespace py = pybind11;


ORBSLAM3Python::ORBSLAM3Python(std::string vocabFile, std::string settingsFile, ORB_SLAM3::System::eSensor sensorMode, bool useViewer, int initFrame, std::string strSequence)
    : vocabluaryFile(vocabFile),
      settingsFile(settingsFile),
      sensorMode(sensorMode),
      system(nullptr),
      bUseViewer(false),
      initFrame(initFrame),
      strSequence(strSequence)
{
    system = std::make_shared<ORB_SLAM3::System>(vocabluaryFile, settingsFile, sensorMode, bUseViewer, initFrame, strSequence);
}

ORBSLAM3Python::~ORBSLAM3Python()
{
}


bool ORBSLAM3Python::TrackStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp, std::vector<ORB_SLAM3::IMU::Point> vImuMeas = vector<ORB_SLAM3::IMU::Point>(), std::string filename = "")
{
    if (!system)
    {
        std::cout << "System not Initialized" << std::endl;
        return false;
    }
    if (leftImage.data && rightImage.data)
    {

        // auto pose = system->TrackStereo(leftImage, rightImage, timestamp, imuData, filename);
        auto pose = system->TrackStereo(leftImage, rightImage, timestamp, vImuMeas, filename);
        return !system->isLost();
    }
    else
    {
        return false;
    }
}


bool ORBSLAM3Python::TrackRGBD(cv::Mat image, cv::Mat depthImage, double timestamp, std::vector<ORB_SLAM3::IMU::Point> vImuMeas = vector<ORB_SLAM3::IMU::Point>(), std::string filename = "")
{
    if (!system)
    {
        std::cout << "Syatem not initialized" << std::endl;
        return false;
    }
    if (image.data && depthImage.data)
    {
        auto pose = system->TrackRGBD(image, depthImage, timestamp, vImuMeas, filename);
        return !system->isLost();
    }
    else
    {
        return false;
    }
}


bool ORBSLAM3Python::TrackMonocular(cv::Mat image, double timestamp, std::vector<ORB_SLAM3::IMU::Point> vImuMeas = std::vector<ORB_SLAM3::IMU::Point>(), std::string filename = "")
{
    if (!system)
    {
        std::cout << "System not initialized" << std::endl;
        return false;
    }
    if (image.data)
    {
        Sophus::SE3f pose = system->TrackMonocular(image, timestamp, vImuMeas, filename);
        return !system->isLost();
    }
    else
    {
        return false;
    }
}


void ORBSLAM3Python::ActivateLocalizationMode()
{
    if (system)
    {
        system->ActivateLocalizationMode();
    }
}


void ORBSLAM3Python::DeactivateLocalizationMode()
{
    if (system)
    {
        system->DeactivateLocalizationMode();
    }
}

bool ORBSLAM3Python::MapChanged()
{
    if (system)
    {
        return system->MapChanged();
    }
    return false;
}

void ORBSLAM3Python::Reset()
{
    if (system)
    {
        system->Reset();
    }
}


void ORBSLAM3Python::ResetActiveMap()
{
    if (system)
    {
        system->ResetActiveMap();
    }
}


void ORBSLAM3Python::Shutdown()
{
    if (system)
    {
        system->Shutdown();
    }
}

bool ORBSLAM3Python::isRunning()
{
    return system != nullptr;
}


void ORBSLAM3Python::setUseViewer(bool useViewer)
{
    bUseViewer = useViewer;
}

void ORBSLAM3Python::SaveTrajectoryTUM(const std::string &filename)
{
    system->SaveTrajectoryTUM(filename);
}


void ORBSLAM3Python::SaveKeyFrameTrajectoryTUM(const std::string &filename)
{
    system->SaveKeyFrameTrajectoryTUM(filename);
}

void ORBSLAM3Python::SaveTrajectoryEuRoC(const std::string &filename)
{
    system->SaveTrajectoryEuRoC(filename);
}

void ORBSLAM3Python::SaveKeyFrameTrajectoryEuRoC(const std::string &filename)
{
    system->SaveKeyFrameTrajectoryEuRoC(filename);
}

void ORBSLAM3Python::SaveDebugData(const int &iniIdx)
{
    system->SaveDebugData(iniIdx);
}

void ORBSLAM3Python::SaveTrajectoryKitti(const std::string &filename)
{
    system->SaveTrajectoryKITTI(filename);
}

int ORBSLAM3Python::GetTrackingState()
{
    return system->GetTrackingState();
}

std::vector<Eigen::Matrix4f> ORBSLAM3Python::GetTrajectory() const
{
    return system->GetCameraTrajectory();
}

std::vector<Eigen::Matrix4f> ORBSLAM3Python::GetFullTrajectory() const
{
    return system->GetFullTrajectory();
}

vector<Eigen::Matrix<float,3,1>> ORBSLAM3Python::GetMapPoints(){
    return system->GetMapPoints();
}

vector<Eigen::Matrix<float,3,1>> ORBSLAM3Python::GetCurrentMapPoints(){
    return system->GetCurrentMapPoints();
}

double ORBSLAM3Python::GetTimeFromIMUInit()
{
    return system->GetTimeFromIMUInit();
}

bool ORBSLAM3Python::isLost()
{
    return system->isLost();
}

bool ORBSLAM3Python::isFinished()
{
    return system->isFinished();
}

void ORBSLAM3Python::ChangeDataset()
{
    system->ChangeDataset();
}

float ORBSLAM3Python::GetImageScale()
{
    return system->GetImageScale();
}

void ORBSLAM3Python::testfunc(ORB_SLAM3::IMU::Point imuData)
{
    std::cout << "a.x: " << imuData.a.x() << " a.y: " << imuData.a.y() << " a.z: " << imuData.a.z() << std::endl;
}

void ORBSLAM3Python::testfunc2(cv::Point3f p)
{
    std::cout << "x: " << p.x << " y: " << p.y << " z: " << p.z << std::endl;
}



PYBIND11_MODULE(orbslam3, m)
{
    m.doc() = "ORB_SLAM3 Python Bindings";

    NDArrayConverter::init_numpy();


    py::enum_<ORB_SLAM3::Tracking::eTrackingState>(m, "TrackingState")
        .value("SYSTEM_NOT_READY", ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY)
        .value("NO_IMAGES_YET", ORB_SLAM3::Tracking::eTrackingState::NO_IMAGES_YET)
        .value("NOT_INITIALIZED", ORB_SLAM3::Tracking::eTrackingState::NOT_INITIALIZED)
        .value("OK", ORB_SLAM3::Tracking::eTrackingState::OK)
        .value("RECENTLY_LOST", ORB_SLAM3::Tracking::eTrackingState::RECENTLY_LOST)
        .value("LOST", ORB_SLAM3::Tracking::eTrackingState::LOST)
        .value("OK_KLT", ORB_SLAM3::Tracking::eTrackingState::OK_KLT);
        
    
    py::class_<cv::Point3f>(m, "Point3f")
        .def(py::init<float, float, float>())
        .def_readwrite("x", &cv::Point3f::x)
        .def_readwrite("y", &cv::Point3f::y)
        .def_readwrite("z", &cv::Point3f::z);


    py::enum_<ORB_SLAM3::System::eSensor>(m, "Sensor")
        .value("MONOCULAR", ORB_SLAM3::System::eSensor::MONOCULAR)
        .value("STEREO", ORB_SLAM3::System::eSensor::STEREO)
        .value("RGBD", ORB_SLAM3::System::eSensor::RGBD)
        .value("IMU_MONOCULAR", ORB_SLAM3::System::eSensor::IMU_MONOCULAR)
        .value("IMU_STEREO", ORB_SLAM3::System::eSensor::IMU_STEREO)
        .value("IMU_RGBD", ORB_SLAM3::System::eSensor::IMU_RGBD);


    py::enum_<ORB_SLAM3::System::FileType>(m, "FileType")
        .value("TEXT_FILE", ORB_SLAM3::System::FileType::TEXT_FILE)
        .value("BINARY_FILE", ORB_SLAM3::System::FileType::BINARY_FILE);


    py::class_<ORB_SLAM3::IMU::Point>(m, "Point")
        .def(py::init<const cv::Point3f &, const cv::Point3f &, const double &>())
        .def(py::init<const float &, const float &, const float &, const float &, const float &, const float &, const double &>())
        .def_readwrite("a", &ORB_SLAM3::IMU::Point::a)
        .def_readwrite("w", &ORB_SLAM3::IMU::Point::w)
        .def_readwrite("t", &ORB_SLAM3::IMU::Point::t);


    py::class_<ORBSLAM3Python>(m, "system")

        .def(py::init<std::string, std::string, ORB_SLAM3::System::eSensor, bool, int, std::string>(), py::arg("vocab_file"), py::arg("settings_file"), py::arg("sensor_type"), py::arg("use_viewer") = false, py::arg("init_frame") = 0, py::arg("sequence") = std::string())


        .def("process_image_stereo", &ORBSLAM3Python::TrackStereo, py::arg("leftImage"), py::arg("rightimage"), py::arg("timestamp"), py::arg("vImuMeas")=std::vector<ORB_SLAM3::IMU::Point>(), py::arg("filename") = std::string())

        .def("process_image_rgbd", &ORBSLAM3Python::TrackRGBD, py::arg("image"), py::arg("depth"), py::arg("time_stamp"), py::arg("vImuMeas")=std::vector<ORB_SLAM3::IMU::Point>(), py::arg("filename") = std::string())


        .def("process_image_mono", &ORBSLAM3Python::TrackMonocular, py::arg("image"), py::arg("timestamp"), py::arg("vImuMeas")=std::vector<ORB_SLAM3::IMU::Point>(), py::arg("filename") = std::string())

        .def("activate_localization_mode", &ORBSLAM3Python::ActivateLocalizationMode)

        .def("deactivate_localization_mode", &ORBSLAM3Python::DeactivateLocalizationMode)

        .def("map_changed", &ORBSLAM3Python::MapChanged)

        .def("reset", &ORBSLAM3Python::Reset)

        .def("reset_active_map", &ORBSLAM3Python::ResetActiveMap)

        .def("shutdown", &ORBSLAM3Python::Shutdown)

        .def("is_running", &ORBSLAM3Python::isRunning)
        
        .def("set_use_viewer", &ORBSLAM3Python::setUseViewer)
        
        .def("save_trajectory_tum", &ORBSLAM3Python::SaveTrajectoryTUM, py::arg("filename"))
        
        .def("save_keyframe_trajectory_tum", &ORBSLAM3Python::SaveKeyFrameTrajectoryTUM, py::arg("filename"))

        .def("save_trajectory_euroc", &ORBSLAM3Python::SaveTrajectoryEuRoC, py::arg("filename"))
        
        .def("save_keyframe_trajectory_euroc", &ORBSLAM3Python::SaveKeyFrameTrajectoryEuRoC, py::arg("filename"))

        .def("save_debug_data", &ORBSLAM3Python::SaveDebugData, py::arg("iniIdx"))

        .def("save_trajectory_kitti", &ORBSLAM3Python::SaveTrajectoryKitti, py::arg("filename"))

        .def("get_tracking_state", &ORBSLAM3Python::GetTrackingState)

        .def("get_trajectory", &ORBSLAM3Python::GetTrajectory)

        .def("get_full_trajectory", &ORBSLAM3Python::GetFullTrajectory)

        .def("get_map_points", &ORBSLAM3Python::GetMapPoints)

        .def("get_current_map_points", &ORBSLAM3Python::GetCurrentMapPoints)

        .def("get_time_from_imu_init", &ORBSLAM3Python::GetTimeFromIMUInit)

        .def("is_lost", &ORBSLAM3Python::isLost)

        .def("is_finished", &ORBSLAM3Python::isFinished)

        .def("change_dataset", &ORBSLAM3Python::ChangeDataset)

        .def("get_image_scale", &ORBSLAM3Python::GetImageScale)

        .def("testfunc", &ORBSLAM3Python::testfunc)
        .def("testfunc2", &ORBSLAM3Python::testfunc2);
}