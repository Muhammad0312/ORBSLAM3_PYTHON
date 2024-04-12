import argparse
from calendar import c
from glob import glob
import os 
import cv2
import time
import pdb

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


slam = orbslam3.system(args.vocab_file, args.settings_file, orbslam3.Sensor.MONOCULAR, False)
imageScale = slam.get_image_scale()


for i in range(len(imgFiles)):
    print("Processing: ", i)
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
    state = slam.get_tracking_state()
    if(state != orbslam3.TrackingState.OK):
        print("System not ready yet")
    if(state == orbslam3.TrackingState.OK):
        # print("System OK")
        allMapPoints = slam.get_map_points()
        currentMapPoints = slam.get_current_map_points()
    
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
# python3 ExamplesPy/mono_euroc.py --vocab_file=extern/ORB_SLAM3/Vocabulary/ORBvoc.txt --settings_file=extern/ORB_SLAM3/Examples/Monocular/EuRoC.yaml --dataset_path=/root/Datasets/EuRoc/MH01 --times_path=extern/ORB_SLAM3/Examples/Monocular/EuRoC_TimeStamps/MH01.txt