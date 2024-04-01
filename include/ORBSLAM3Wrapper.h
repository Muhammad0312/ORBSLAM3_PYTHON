#ifndef ORB_SLAM3_PYTHON_H
#define ORB_SLAM3_PYTHON_H

#include <memory>
#include <System.h>
#include <Tracking.h>

class ORBSLAM3Python
{
public:
    ORBSLAM3Python(std::string vocabFile, std::string settingsFile,
                   ORB_SLAM3::System::eSensor sensorMode, bool useViewer = false, int initFrame = 0, std::string strSequence = std::string());
    ~ORBSLAM3Python();

    bool initialize();
    bool processMono(cv::Mat image, double timestamp, std::vector<ORB_SLAM3::IMU::Point> vImuMeas, std::string filename);
    bool processStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp, std::vector<ORB_SLAM3::IMU::Point> vImuMeas, std::string filename);
    bool processRGBD(cv::Mat image, cv::Mat depthImage, double timestamp);
    void reset();
    void shutdown();
    bool isRunning();
    void setUseViewer(bool useViewer);
    std::vector<Eigen::Matrix4f> getTrajectory() const;
    // void testfunc(ORB_SLAM3::IMU::Point imuData);
    // void testfunc2(cv::Point3f p);

private:
    std::string vocabluaryFile;
    std::string settingsFile;
    ORB_SLAM3::System::eSensor sensorMode;
    std::shared_ptr<ORB_SLAM3::System> system;
    bool bUseViewer;
    bool bUseRGB;
    int initFrame;
    std::string strSequence;
};

#endif // ORB_SLAM3_PYTHON_H