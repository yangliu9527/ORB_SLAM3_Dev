# python3 ./Eva/run_exp_tracking_OMD.py --cam_pose=1 --obj_pose=0 --mot=0 --plot=0 --f50=1
import os
import argparse
import re

parser = argparse.ArgumentParser()
parser.add_argument("--cam_pose",type=int,default=1)
parser.add_argument("--plot",type=int,default=1)
parser.add_argument("--seq_id",type=str)
parser.add_argument("--frame_num",type=int,default=500)

########################evaluation parameters########################
eva_campose = parser.parse_args().cam_pose
plot_flag = parser.parse_args().plot
seq_id = parser.parse_args().seq_id
frame_num = parser.parse_args().frame_num

print(f'evaluate cam_pose:{eva_campose}, plot figures:{plot_flag}')

########################dataset and workspace path########################
dataset_path = "/home/zhiyu/DataSet/TUM-RGBD/"
evaluation_workspace = "./Evaluation/TUM-RGBD/"

#seq_ids =["rgbd_dataset_freiburg3_walking_xyz", "rgbd_dataset_freiburg2_desk"]
seq_ids =["rgbd_dataset_freiburg3_walking_xyz", "rgbd_dataset_freiburg3_walking_static","rgbd_dataset_freiburg3_walking_halfsphere"]

# select seq ids
if (seq_id == None):
    seq_ids = seq_ids
else:
    seq_ids = [seq_id]


# evaluation
for seq_id in seq_ids:
    #######################run the system#######################
    numbers = re.findall(r'\d+', seq_id)
    numbers = [int(num) for num in numbers]
    print(str(numbers[0]))
    os.system(f'./Examples_added/RGB-D/rgbd_tum_added Vocabulary/ORBvoc.txt ./Config/TUM-RGBD/TUM{str(numbers[0])}.yaml  {dataset_path}/{seq_id}/ {dataset_path}/{seq_id}/associate.txt')
    os.system(f'cp CameraTrajectory.txt {evaluation_workspace}/results/{seq_id}_est.txt')
    print(f"#############################evaluating {seq_id} camera pose#############################")
    os.system(f'evo_traj tum {evaluation_workspace}/results/{seq_id}_est.txt --ref={evaluation_workspace}/ground_truth/{seq_id}_gt.txt -a --no_warnings    --plot_mode=xy  --save_plot {evaluation_workspace}/results/{seq_id}_evo_traj')
    os.system(f'evo_ape tum {evaluation_workspace}/ground_truth/{seq_id}_gt.txt {evaluation_workspace}/results/{seq_id}_est.txt -a --no_warnings  --save_plot {evaluation_workspace}/results/{seq_id}_evo_ate')  