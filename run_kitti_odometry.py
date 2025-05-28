import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--seq_id", type=str, default=None)

######################## evaluation parameters########################
seq_id = parser.parse_args().seq_id
######################## dataset and workspace path########################
dataset_path = "/home/brain/DataSet/KITTI-Odometry/"
evaluation_workspace = "/home/brain/LYCodes/ORB_SLAM3_Dev/Evaluation/KITTI-Odometry"

# seq ids for all evlaution
all_seq_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

# select seq ids
if (seq_id == None):
    seq_ids = all_seq_ids
else:
    seq_ids = [int(seq_id)]
print(f'seq ids for evalaution: {seq_ids}')


# evaluation
for i in seq_ids:
    seq_id = str(i).zfill(2)
    ####################### run the system#######################
    os.system(
        f'./Examples_added/Stereo/stereo_kitti_added Vocabulary/ORBvoc.txt  ./Config/KITTI-Odometry/KITTI{seq_id}.yaml {dataset_path}/{seq_id}/')
    os.system(
        f'cp CameraTrajectory.txt {evaluation_workspace}/results/est_{seq_id}.txt')
    os.system(
        f'python ./Evaluation/KITTI_Tools/evaluate_kitti.py --gt_file={evaluation_workspace}/ground_truth/{seq_id}.txt  --est_file={evaluation_workspace}/results/est_{seq_id}.txt --output_dir={evaluation_workspace}/results/ --align=6dof --seq_id={i}')
    