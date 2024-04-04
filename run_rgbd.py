from lib import orbslam3
import argparse
from glob import glob
import os 
import cv2
import pdb

parser = argparse.ArgumentParser()
parser.add_argument("--vocab_file", required=True)
parser.add_argument("--settings_file", required=True)
parser.add_argument("--dataset_path", required=True)
args = parser.parse_args()

img_files = sorted(glob(os.path.join(args.dataset_path, 'rgb/*.png')))
slam = orbslam3.system(args.vocab_file, args.settings_file, orbslam3.Sensor.MONOCULAR, False)

print(type(args.vocab_file))
print(type(args.settings_file))
print(type(orbslam3.Sensor.MONOCULAR))
print("SLAM object created")

# slam.set_use_viewer(False)
# slam.initialize()

# slam.testfunc2(orbslam3.Point3f(1.0,2.0,3.0))

# for img in img_files:
#     timestamp = img.split('/')[-1][:-4]
#     img = cv2.imread(img, -1)
#     pose = slam.process_image_mono(img, float(timestamp))
#     # print(pose)

# print(slam.get_trajectory())


# Run the script with the following command:
# python3 run_rgbd.py --vocab_file=extern/ORB_SLAM3/Vocabulary/ORBvoc.txt --settings_file=extern/ORB_SLAM3/Examples/RGB-D/TUM1.yaml --dataset_path=/root/Datasets/TUM/rgbd_dataset_freiburg1_xyz