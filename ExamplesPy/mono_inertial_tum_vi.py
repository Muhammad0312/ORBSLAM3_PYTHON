import argparse
from calendar import c
from glob import glob
import os
from turtle import pd 
import cv2
import time
import pdb

# Add the path to lib folder to PYTHONPATH

import orbslam3

parser = argparse.ArgumentParser()
parser.add_argument("--vocab_file", required=True, help="Path to ORB vocabulary file")
parser.add_argument("--settings_file", required=True, help="Path to settings file")
parser.add_argument("--dataset_path", required=True, help="Path to sequence folder")
parser.add_argument("--times_path", required=True, help="Path to times file")
parser.add_argument("--output_path", required=False, help="Output trajectory file name")
args = parser.parse_args()

# Load the images
imgFiles = sorted(glob(os.path.join(args.dataset_path, 'mav0/cam0/data/*.png')))

# Load the timestamps
timeStamps = []
with open(args.times_path, 'r') as f:
    for line in f:
        timeStamps.append(int(line.split()[0])*1e-9)



imuFile = os.path.join(args.dataset_path, 'mav0/imu0/data.csv')
# Load IMU data
imuTimeStamps = []
accData = []
gyroData = []
with open(imuFile, 'r') as f:
    for line in f:
        data = line.split(',')
        if data[0] == '#timestamp [ns]':
            continue
        imuTimeStamps.append(int(data[0])*1e-9)
        accData.append(orbslam3.Point3f(float(data[4]), float(data[5]), float(data[6])))
        gyroData.append(orbslam3.Point3f(float(data[1]), float(data[2]), float(data[3])))


# First Imu measurement which matches the first image timestamp
firstImu = 0
while imuTimeStamps[firstImu] <= timeStamps[0]:
    firstImu += 1
firstImu -= 1


slam = orbslam3.system(args.vocab_file, args.settings_file, orbslam3.Sensor.IMU_MONOCULAR, True)
imageScale = slam.get_image_scale()

clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))


for i in range(len(imgFiles)):
    print("Processing: ", i)
    vImuMeas = []
    startTime = time.time()
    currentTimestamp = timeStamps[i]
    img = cv2.imread(imgFiles[i], cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("Failed to load image at path: ", img)
        break

    if imageScale != 1.0:
        width = img.cols * imageScale
        height = img.rows * imageScale
        img = cv2.resize(img, (width, height))

    if i > 0:
        while imuTimeStamps[firstImu] <= currentTimestamp:
            vImuMeas.append(orbslam3.Point(accData[firstImu].x, accData[firstImu].y, accData[firstImu].z, gyroData[firstImu].x, gyroData[firstImu].y, gyroData[firstImu].z, imuTimeStamps[firstImu]))
            firstImu += 1

    img = clahe.apply(img)

    pose = slam.process_image_mono(img, currentTimestamp, vImuMeas)
    endTime = time.time()

    # If processing is faster than real-time, sleep for a while
    if i + 1 < len(timeStamps):
        nextTimestamp = timeStamps[i+1]
    else:
        nextTimestamp = currentTimestamp + 0.1

    frameTime = (nextTimestamp - currentTimestamp)
    processingTime = endTime - startTime
    if processingTime < frameTime:
        time.sleep(frameTime - processingTime)

slam.shutdown()

if args.output_path:
    print("Saving trajectory to: ", args.output_path)
    slam.save_trajectory_euroc(args.output_path + ".txt")
    slam.save_keyframe_trajectory_euroc(args.output_path + "_keyframe.txt")
else:
    slam.save_trajectory_euroc("CameraTrajectory.txt")
    slam.save_keyframe_trajectory_euroc("KeyFrameTrajectory.txt")


# Run the script with the following command:
# python3 ExamplesPy/mono_inertial_tum_vi.py --vocab_file=extern/ORB_SLAM3/Vocabulary/ORBvoc.txt --settings_file=extern/ORB_SLAM3/Examples/Monocular-Inertial/TUM-VI.yaml --dataset_path=/root/Datasets/TUM_VI/dataset-corridor1_512_16 --times_path=extern/ORB_SLAM3/Examples/Monocular-Inertial/TUM_TimeStamps/dataset-corridor1_512.txt