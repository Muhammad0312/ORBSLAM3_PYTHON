import argparse
from glob import glob
import os 
import cv2
import sys


# Get the current directory of the script
current_dir = os.path.dirname(os.path.realpath(__file__))

# Append the 'lib' directory to the Python path
lib_dir = os.path.join(current_dir, '..', 'lib')
sys.path.append(lib_dir)

import orbslam3

parser = argparse.ArgumentParser()
parser.add_argument("--vocab_file", required=True)
parser.add_argument("--settings_file", required=True)
parser.add_argument("--dataset_path", required=True)
args = parser.parse_args()

img_files = sorted(glob(os.path.join(args.dataset_path, 'rgb/*.png')))
slam = orbslam3.system(args.vocab_file, args.settings_file, orbslam3.Sensor.MONOCULAR)
slam.set_use_viewer(True)
slam.initialize()

for img in img_files:
    timestamp = img.split('/')[-1][:-4]
    img = cv2.imread(img, -1)
    pose = slam.process_image_mono(img, float(timestamp))
    print(pose)

# Run the script with the following command:
# python3 run_rgbd.py --vocab_file=../extern/ORB_SLAM3/Vocabulary/ORBvoc.txt --settings_file=../extern/ORB_SLAM3/Examples/RGB-D/TUM1.yaml --dataset_path=/root/Datasets/TUM/rgbd_dataset_freiburg1_xyz