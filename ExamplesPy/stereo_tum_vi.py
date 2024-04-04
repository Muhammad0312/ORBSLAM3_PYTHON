import argparse
from glob import glob
import os
from turtle import pd 
import cv2
import time
import pdb

from numpy import double

# Add the path to lib folder to PYTHONPATH

import orbslam3

parser = argparse.ArgumentParser()
parser.add_argument("--vocab_file", required=True, help="Path to ORB vocabulary file")
parser.add_argument("--settings_file", required=True, help="Path to settings file")
parser.add_argument("--dataset_path_1", required=True, help="Path to sequence folder cam1")
parser.add_argument("--dataset_path_2", required=True, help="Path to sequence folder cam2")
parser.add_argument("--times_path", required=True, help="Path to times file")
parser.add_argument("--output_path", required=False, help="Output trajectory file name")
args = parser.parse_args()

# Load the images
timeStampsFile = args.times_path

# Load the images
imgFilesLeft = sorted(glob(os.path.join(args.dataset_path_1, '*.png')))
imgFilesRight = sorted(glob(os.path.join(args.dataset_path_2, '*.png')))
# Load the timestamps
timeStamps = []

with open(timeStampsFile, 'r') as f:
    for line in f:
        l = line.split()
        timeStamps.append(double(l[0])*1e-9)


slam = orbslam3.system(args.vocab_file, args.settings_file, orbslam3.Sensor.STEREO, False)
imageScale = slam.get_image_scale()

clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

for i in range(len(imgFilesLeft)):
    print("Processing: ", i)
    startTime = time.time()
    currentTimestamp = timeStamps[i]
    imgLeft = cv2.imread(imgFilesLeft[i], cv2.IMREAD_GRAYSCALE)
    imgRight = cv2.imread(imgFilesRight[i], cv2.IMREAD_GRAYSCALE)
    if imgLeft is None:
        print("Failed to load image at path: ", imgLeft)
        break

    if imgRight is None:
        print("Failed to load image at path: ", imgRight)
        break

    if imageScale != 1.0:
        width = imgLeft.cols * imageScale
        height = imgLeft.rows * imageScale
        imgLeft = cv2.resize(imgLeft, (width, height))
        imgRight = cv2.resize(imgRight, (width, height))

    imgLeft = clahe.apply(imgLeft)
    imgRight = clahe.apply(imgRight)

    pose = slam.process_image_stereo(imgLeft, imgRight, currentTimestamp)
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
    slam.save_trajectory_euroc(args.output_path + ".txt")
    slam.save_keyframe_trajectory_euroc(args.output_path + "_keyframe.txt")
else:
    slam.save_trajectory_euroc("CameraTrajectory.txt")
    slam.save_keyframe_trajectory_euroc("KeyFrameTrajectory.txt")


# Run the script with the following command:
# python3 ExamplesPy/stereo_tum_vi.py --vocab_file=extern/ORB_SLAM3/Vocabulary/ORBvoc.txt --settings_file=extern/ORB_SLAM3/Examples/Stereo/TUM-VI.yaml --dataset_path_1=/root/Datasets/TUM_VI/dataset-corridor1_512_16/mav0/cam0/data --dataset_path_2=/root/Datasets/TUM_VI/dataset-corridor1_512_16/mav0/cam1/data --times_path=extern/ORB_SLAM3/Examples/Stereo/TUM_TimeStamps/dataset-corridor1_512.txt