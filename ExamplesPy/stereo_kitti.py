import argparse
from glob import glob
import os 
import cv2
import time
import pdb

from numpy import double

# Add the path to lib folder to PYTHONPATH

import orbslam3

parser = argparse.ArgumentParser()
parser.add_argument("--vocab_file", required=True, help="Path to ORB vocabulary file")
parser.add_argument("--settings_file", required=True, help="Path to settings file")
parser.add_argument("--dataset_path", required=True, help="Path to sequence folder")
parser.add_argument("--output_path", required=False, help="Output trajectory file name")
args = parser.parse_args()

# Load the images
imgFilesLeft = sorted(glob(os.path.join(args.dataset_path, 'image_0/*.png')))
imgFilesRight = sorted(glob(os.path.join(args.dataset_path, 'image_1/*.png')))
timeStampsFile = os.path.join(args.dataset_path, 'times.txt')


# Load the timestamps
timeStamps = []
with open(timeStampsFile, 'r') as f:
    for line in f:
        timeStamps.append(double(line.split()[0]))


slam = orbslam3.system(args.vocab_file, args.settings_file, orbslam3.Sensor.STEREO, False)
imageScale = slam.get_image_scale()

for i in range(len(imgFilesLeft)):
    # print("Processing Frame: ", i)
    startTime = time.time()
    currentTimestamp = timeStamps[i]
    imgLeft = cv2.imread(imgFilesLeft[i], cv2.IMREAD_UNCHANGED)
    imgRight = cv2.imread(imgFilesRight[i], cv2.IMREAD_UNCHANGED)

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

    pose = slam.process_image_stereo(imgLeft, imgRight, currentTimestamp)
    state = slam.get_tracking_state()
    if(state != orbslam3.TrackingState.OK):
        print("System not ready yet")
    
    allMapPoints = slam.get_map_points()
    currentMapPoints = slam.get_current_map_points()
    slam.get_full_trajectory()
    endTime = time.time()

    # If processing is faster than real-time, sleep for a while
    if i + 1 < len(timeStamps):
        nextTimestamp = timeStamps[i+1]
    else:
        nextTimestamp = currentTimestamp + 0.1

    frameTime = (nextTimestamp - currentTimestamp)
    processingTime = endTime - startTime
    if processingTime < frameTime:
        print("Sleeping for: ", frameTime - processingTime)
        time.sleep(frameTime - processingTime)
    else:
        print("Processing Time Exceeded Frame Time")

slam.shutdown()

if args.output_path:
    slam.save_trajectory_kitti(args.output_path + "_keyframe.txt")
else:
    slam.save_trajectory_kitti("KeyFrameTrajectory.txt")


# Run the script with the following command:
# python3 ExamplesPy/stereo_kitti.py --vocab_file=extern/ORB_SLAM3/Vocabulary/ORBvoc.txt --settings_file=extern/ORB_SLAM3/Examples/Stereo/KITTI00-02.yaml --dataset_path=/root/Datasets/KITTI/dataset/sequences/00