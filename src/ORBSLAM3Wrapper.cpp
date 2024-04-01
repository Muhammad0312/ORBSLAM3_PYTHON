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
}

ORBSLAM3Python::~ORBSLAM3Python()
{
}

bool ORBSLAM3Python::initialize()
{
    system = std::make_shared<ORB_SLAM3::System>(vocabluaryFile, settingsFile, sensorMode, bUseViewer, initFrame, strSequence);
    return true;
}

bool ORBSLAM3Python::isRunning()
{
    return system != nullptr;
}

void ORBSLAM3Python::reset()
{
    if (system)
    {
        system->Reset();
    }
}

bool ORBSLAM3Python::processMono(cv::Mat image, double timestamp, std::vector<ORB_SLAM3::IMU::Point> vImuMeas = std::vector<ORB_SLAM3::IMU::Point>(), std::string filename = "")
{
    if (!system)
    {
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

bool ORBSLAM3Python::processStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp, std::vector<ORB_SLAM3::IMU::Point> vImuMeas = vector<ORB_SLAM3::IMU::Point>(), std::string filename = "")
{
    if (!system)
    {
        std::cout << "you must call initialize() first!" << std::endl;
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

bool ORBSLAM3Python::processRGBD(cv::Mat image, cv::Mat depthImage, double timestamp)
{
    if (!system)
    {
        std::cout << "you must call initialize() first!" << std::endl;
        return false;
    }
    if (image.data && depthImage.data)
    {
        auto pose = system->TrackRGBD(image, depthImage, timestamp);
        return !system->isLost();
    }
    else
    {
        return false;
    }
}

void ORBSLAM3Python::shutdown()
{
    if (system)
    {
        system->Shutdown();
    }
}

void ORBSLAM3Python::setUseViewer(bool useViewer)
{
    bUseViewer = useViewer;
}

std::vector<Eigen::Matrix4f> ORBSLAM3Python::getTrajectory() const
{
    return system->GetCameraTrajectory();
}

// void ORBSLAM3Python::testfunc(ORB_SLAM3::IMU::Point imuData)
// {
//     std::cout << "a.x: " << imuData.a.x() << " a.y: " << imuData.a.y() << " a.z: " << imuData.a.z() << std::endl;
// }

// void ORBSLAM3Python::testfunc2(cv::Point3f p)
// {
//     std::cout << "x: " << p.x << " y: " << p.y << " z: " << p.z << std::endl;
// }



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

    py::class_<ORB_SLAM3::IMU::Point>(m, "Point")
        .def(py::init<const cv::Point3f &, const cv::Point3f &, const double &>())
        .def(py::init<const float &, const float &, const float &, const float &, const float &, const float &, const double &>())
        .def_readwrite("a", &ORB_SLAM3::IMU::Point::a)
        .def_readwrite("w", &ORB_SLAM3::IMU::Point::w)
        .def_readwrite("t", &ORB_SLAM3::IMU::Point::t);


    py::class_<ORBSLAM3Python>(m, "system")
        .def(py::init<std::string, std::string, ORB_SLAM3::System::eSensor, bool, int, std::string>(), py::arg("vocab_file"), py::arg("settings_file"), py::arg("sensor_type"), py::arg("use_viewer") = false, py::arg("init_frame") = 0, py::arg("sequence") = std::string())

        .def("initialize", &ORBSLAM3Python::initialize)

        .def("process_image_mono", &ORBSLAM3Python::processMono, py::arg("image"), py::arg("timestamp"), py::arg("vImuMeas")=std::vector<ORB_SLAM3::IMU::Point>(), py::arg("filename") = std::string())

        .def("process_image_stereo", &ORBSLAM3Python::processStereo, py::arg("leftImage"), py::arg("rightimage"), py::arg("timestamp"), py::arg("vImuMeas"), py::arg("filename") = std::string())

        .def("process_image_rgbd", &ORBSLAM3Python::processRGBD, py::arg("image"), py::arg("depth"), py::arg("time_stamp"))

        .def("shutdown", &ORBSLAM3Python::shutdown)
        .def("is_running", &ORBSLAM3Python::isRunning)
        .def("reset", &ORBSLAM3Python::reset)
        .def("set_use_viewer", &ORBSLAM3Python::setUseViewer)
        .def("get_trajectory", &ORBSLAM3Python::getTrajectory);
        // .def("testfunc", &ORBSLAM3Python::testfunc)
        // .def("testfunc2", &ORBSLAM3Python::testfunc2);
}