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
parser.add_argument("--association_path", required=True, help="Path to associations file")
parser.add_argument("--output_path", required=False, help="Output trajectory file name")
args = parser.parse_args()

# Load the images
associationFile = args.association_path

# Image file paths
imgFiles = []
# Depth file paths
depthFiles = []
# Load the timestamps
timeStamps = []

with open(associationFile, 'r') as f:
    for line in f:
        l = line.split()
        timeStamps.append(double(l[0]))
        imgFiles.append(os.path.join(args.dataset_path, l[1]))
        depthFiles.append(os.path.join(args.dataset_path, l[3]))

if len(imgFiles) == 0:
    print("No images found in the provided association file.")
    exit()
elif len(imgFiles) != len(depthFiles):
    print("Number of images and depth maps do not match.")
    exit()


slam = orbslam3.system(args.vocab_file, args.settings_file, orbslam3.Sensor.RGBD, False)
imageScale = slam.get_image_scale()

for i in range(len(imgFiles)):
    print("Processing Frame: ", i)
    startTime = time.time()
    currentTimestamp = timeStamps[i]
    imgRGB = cv2.imread(imgFiles[i], cv2.IMREAD_UNCHANGED)
    imgD = cv2.imread(depthFiles[i], cv2.IMREAD_UNCHANGED)

    if imgRGB is None:
        print("Failed to load image at path: ", imgRGB)
        break

    if imageScale != 1.0:
        width = imgRGB.cols * imageScale
        height = imgRGB.rows * imageScale
        imgRGB = cv2.resize(imgRGB, (width, height))
        imgD = cv2.resize(imgD, (width, height))

    pose = slam.process_image_rgbd(imgRGB, imgD, currentTimestamp)
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
    slam.save_keyframe_trajectory_tum(args.output_path + "_keyframe.txt")
else:
    slam.save_keyframe_trajectory_tum("KeyFrameTrajectory.txt")


# Run the script with the following command:
# python3 ExamplesPy/rgbd_tum.py --vocab_file=extern/ORB_SLAM3/Vocabulary/ORBvoc.txt --settings_file=extern/ORB_SLAM3/Examples/RGB-D/TUM1.yaml --dataset_path=/root/Datasets/TUM/rgbd_dataset_freiburg1_desk --association_path=extern/ORB_SLAM3/Examples/RGB-D/associations/fr1_desk.txt