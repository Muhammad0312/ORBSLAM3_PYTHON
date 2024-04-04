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
timeStampsFile = os.path.join(args.dataset_path, 'rgb.txt')

# Image file paths
imgFiles = []
# Load the timestamps
timeStamps = []

with open(timeStampsFile, 'r') as f:
    numLine = 0
    for line in f:
        if numLine < 3:
            numLine += 1
            continue
        l = line.split()
        timeStamps.append(double(l[0]))
        imgFiles.append(os.path.join(args.dataset_path, l[1]))


slam = orbslam3.system(args.vocab_file, args.settings_file, orbslam3.Sensor.MONOCULAR, False)
imageScale = slam.get_image_scale()

for i in range(len(imgFiles)):
    print("Processing Frame: ", i)
    startTime = time.time()
    currentTimestamp = timeStamps[i]
    img = cv2.imread(imgFiles[i], cv2.IMREAD_UNCHANGED)
    if img is None:
        print("Failed to load image at path: ", img)
        break

    if imageScale != 1.0:
        width = img.cols * imageScale
        height = img.rows * imageScale
        img = cv2.resize(img, (width, height))

    pose = slam.process_image_mono(img, currentTimestamp)
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
# python3 ExamplesPy/mono_tum.py --vocab_file=extern/ORB_SLAM3/Vocabulary/ORBvoc.txt --settings_file=extern/ORB_SLAM3/Examples/Monocular/TUM1.yaml --dataset_path=/root/Datasets/TUM/rgbd_dataset_freiburg1_xyz