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

    bool TrackStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp, std::vector<ORB_SLAM3::IMU::Point> vImuMeas, std::string filename);

    bool TrackRGBD(cv::Mat image, cv::Mat depthImage, double timestamp, std::vector<ORB_SLAM3::IMU::Point> vImuMeas, std::string filename);

    bool TrackMonocular(cv::Mat image, double timestamp, std::vector<ORB_SLAM3::IMU::Point> vImuMeas, std::string filename);

    void ActivateLocalizationMode();
    void DeactivateLocalizationMode();
    bool MapChanged();

    void Reset();

    void ResetActiveMap();

    void Shutdown();
    bool isRunning();
    void setUseViewer(bool useViewer);

    void SaveTrajectoryTUM(const std::string &filename);
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    void SaveTrajectoryEuRoC(const string &filename);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename);

    // // void SaveTrajectoryEuRoC(const string &filename, Map* pMap);
    // // void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap);

    void SaveDebugData(const int &iniIdx);

    void SaveTrajectoryKitti(const std::string &filename);

    int GetTrackingState();

    // std::vector<MapPoint*> GetTrackedMapPoints();
    // std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();
    std::vector<Eigen::Matrix4f> GetTrajectory() const;

    std::vector<Eigen::Matrix4f> GetFullTrajectory() const;

    vector<Eigen::Matrix<float,7,1>> GetMapPoints();

    vector<Eigen::Matrix<float,7,1>> GetCurrentMapPoints();

    int GetNumKeyFrames();

    std::vector<std::vector<int>> GetKeyFrameIds();

    std::vector<int> GetCovisibleFrameIds(int KeyFrameId, int NumKeyFrames);

    int GetLatestKeyFrameId();

    double GetTimeFromIMUInit();
    bool isLost();
    bool isFinished();

    void ChangeDataset();

    float GetImageScale();

    bool isKeyFrame();

    void testfunc(ORB_SLAM3::IMU::Point imuData);
    void testfunc2(cv::Point3f p);

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