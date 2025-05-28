import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--seq_id", type=str, default=None)

######################## evaluation parameters########################
seq_id = parser.parse_args().seq_id
######################## dataset and workspace path########################
dataset_path = "/home/brain/DataSet/SelfCollected/LYR_AGV/"
evaluation_workspace = "/home/brain/LYCodes/ORB_SLAM3_Dev/Evaluation/LYR-AGV"

# seq ids for all evlaution
#all_seq_ids = ["road1_cut_20250112","road2_cut_20250112","playground_circle_cut_20250112","data1_checked_20250112","playground_cut_20241227"]
all_seq_ids = ["road1_cut_20250305","road2_cut_20250305","road3_cut_20250305","playground_cut_20250305"]
# select seq ids
if (seq_id == None):
    seq_ids = all_seq_ids
else:
    seq_ids = [seq_id]
print(f'seq ids for evalaution: {seq_ids}')


# evaluation
for seq_name in seq_ids:
    ####################### run the system#######################
    print(f'==============================Evaluating {seq_name} ============================')
    date = seq_name.split('_')[-1]
    os.system(f'./Examples_added/Stereo/stereo_kitti_added Vocabulary/ORBvoc.txt  ./Config/LYR-AGV/LYR-AGV-{date}-rectified.yaml {dataset_path}/{seq_name}/')
    os.system(f'cp CameraTrajectory.txt {evaluation_workspace}/results/{seq_name}_est.txt')
    os.system(f'python ./Evaluation/KITTI_Tools/evaluate_kitti.py --gt_file={evaluation_workspace}/ground_truth/{seq_name}_gt.txt  --est_file={evaluation_workspace}/results/{seq_name}_est.txt --output_dir={evaluation_workspace}/results/ --align=6dof --seq_id={seq_name}')
    os.system(f'evo_traj kitti {evaluation_workspace}/results/{seq_name}_est.txt --ref={evaluation_workspace}/ground_truth/{seq_name}_gt.txt -a --no_warnings    --plot_mode=xy  --save_plot {evaluation_workspace}/results/{seq_name}_evo_traj')
    os.system(f'evo_ape kitti {evaluation_workspace}/ground_truth/{seq_name}_gt.txt {evaluation_workspace}/results/{seq_name}_est.txt -a --no_warnings  --save_plot {evaluation_workspace}/results/{seq_name}_evo_ate')  