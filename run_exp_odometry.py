import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--eva_type",type=str,default="both")
parser.add_argument("--plot",type=int,default=0)
eva_type = parser.parse_args().eva_type
plot_flag = parser.parse_args().plot

dataset_path = "/home/zhiyu/DataSet/KITTI-Odometry-Full/"
evaluation_workspace = "eval_kitti_odo/"
for i in range(0,2):
    yaml_file =" "
    if i <=2:
        yaml_file = f'KITTI00-02.yaml'
    elif i==3:
        yaml_file = f'KITTI03.yaml'
    elif i>=4 and i<=12:
        yaml_file = f'KITTI04-12.yaml'

    seq_id = str(i).zfill(2)
    os.system(f'./Examples_added/Stereo/stereo_kitti_added Vocabulary/ORBvoc.txt  Examples_added/Stereo/{yaml_file} {dataset_path}/{seq_id}/')
    os.system(f'cp CameraTrajectory.txt {evaluation_workspace}/eval_results/CameraTrajectory_{seq_id}.txt')
    if(plot_flag):
        os.system(f'evo_ape kitti {evaluation_workspace}/groundtruth/{seq_id}.txt {evaluation_workspace}/eval_results/CameraTrajectory_{seq_id}.txt   --no_warnings  --plot_mode=xz -va --save_plot {evaluation_workspace}/eval_results/{seq_id}_ape --save_results {evaluation_workspace}/eval_results/{seq_id}_ape.zip')
        os.system(f'evo_rpe kitti {evaluation_workspace}/groundtruth/{seq_id}.txt {evaluation_workspace}/eval_results/CameraTrajectory_{seq_id}.txt   --no_warnings  -r trans_part   -va --save_plot {evaluation_workspace}/eval_results/{seq_id}_rpe_t --save_results {evaluation_workspace}/eval_results/{seq_id}_rpe_t.zip')
        os.system(f'evo_rpe kitti {evaluation_workspace}/groundtruth/{seq_id}.txt {evaluation_workspace}/eval_results/CameraTrajectory_{seq_id}.txt   --no_warnings  -r angle_deg   -va --save_plot {evaluation_workspace}/eval_results/{seq_id}_rpe_R --save_results {evaluation_workspace}/eval_results/{seq_id}_rpe_R.zip')
        os.system(f'evo_traj kitti {evaluation_workspace}/eval_results/CameraTrajectory_{seq_id}.txt --ref={evaluation_workspace}/groundtruth/{seq_id}.txt --no_warnings   --plot_mode=xz  --save_plot {evaluation_workspace}/eval_results/{seq_id}_traj')
    else:
        os.system(f'evo_ape kitti {evaluation_workspace}/groundtruth/{seq_id}.txt {evaluation_workspace}/eval_results/CameraTrajectory_{seq_id}.txt   --no_warnings   -va  --save_results {evaluation_workspace}/eval_results/{seq_id}_ape.zip')
        os.system(f'evo_rpe kitti {evaluation_workspace}/groundtruth/{seq_id}.txt {evaluation_workspace}/eval_results/CameraTrajectory_{seq_id}.txt   --no_warnings  -r trans_part   -va  --save_results {evaluation_workspace}/eval_results/{seq_id}_rpe_t.zip')
        os.system(f'evo_rpe kitti {evaluation_workspace}/groundtruth/{seq_id}.txt {evaluation_workspace}/eval_results/CameraTrajectory_{seq_id}.txt   --no_warnings  -r angle_deg   -va  --save_results {evaluation_workspace}/eval_results/{seq_id}_rpe_R.zip')

    os.system(f'unzip -q -o {evaluation_workspace}/eval_results/{seq_id}_ape.zip -d {evaluation_workspace}/eval_results/{seq_id}_ape')
    os.system(f'unzip -q -o {evaluation_workspace}/eval_results/{seq_id}_rpe_t.zip -d {evaluation_workspace}/eval_results/{seq_id}_rpe_t')
    os.system(f'unzip -q -o {evaluation_workspace}/eval_results/{seq_id}_rpe_R.zip -d {evaluation_workspace}/eval_results/{seq_id}_rpe_R')
    
