# python3 run_exp_tracking.py --cam_pose=1 --obj_pose=1 --mot=1 --plot=0 --seq_id=3
import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--seq_id", type=str, default=None)

######################## evaluation parameters########################
seq_id = parser.parse_args().seq_id
######################## dataset and workspace path########################
dataset_path = "/home/brain/DataSet/KITTI-Tracking/"
evaluation_workspace = "./Evaluation/KITTI-Tracking/"

# correspondences from gt_object_id to est_object_id
obj_gt_est_relation = {3:[[1,2]],5:[[31,1]],10:[[0,1]],11:[[0,4],[35,88]],18:[[2,7],[3,5]],19:[[63,41],[72,50]],20:[[0,3],[12,39],[122,308]]} #3:[[0,1]] means obj_0_gt --> obj_1_est

# seq ids for all evlaution
all_seq_ids = [3,5,10,11,18,19,20]

# select seq ids
if (seq_id == None):
    seq_ids = all_seq_ids
else:
    seq_ids = [int(seq_id)]
print(f'seq ids for evalaution: {seq_ids}')


# evaluation
for i in seq_ids:
    seq_id = str(i).zfill(4)
    ####################### run the system#######################
    print(f"*******************************Runing {seq_id} sequence*******************************")
    os.system(f'./Examples_added/Stereo/stereo_kitti_added Vocabulary/ORBvoc.txt ./Config/KITTI-Tracking/best_23/KITTI{seq_id}.yaml {dataset_path}/{seq_id}/')
    #==========evaluate camera pose estimation==========
    print(f"=======================================evaluating {seq_id} camera pose=======================================")
    os.system(f'cp CameraTrajectory.txt {evaluation_workspace}/results_ego/est_{seq_id}.txt')
    os.system(f'python ./Evaluation/KITTI_Tools/evaluate_kitti.py --gt_file={evaluation_workspace}/ground_truth_ego/pose_gt_{seq_id}.txt  --est_file={evaluation_workspace}/results_ego/est_{seq_id}.txt --output_dir={evaluation_workspace}/results_ego/ --align=6dof --seq_id={i}')
    #==========evaluate object pose estimation==========
    os.system(f'cp ObjectTrajectory.txt {evaluation_workspace}/results_object/pose_est_{seq_id}.txt')
    if(int(seq_id) in obj_gt_est_relation):
        relations = obj_gt_est_relation[int(seq_id)]
        for relation in relations:
                gt_trackid = relation[0]
                est_trackid = relation[1]
                print(f"=======================================evaluating {seq_id} object {gt_trackid} pose=======================================")
                os.system(f'python {evaluation_workspace}/KITTI_Tracking_Tools/extract_objtraj.py {evaluation_workspace}/results_object/pose_est_{seq_id}.txt {seq_id} {est_trackid} {evaluation_workspace}/results_object/pose_est_{seq_id}_{est_trackid}.txt')
                os.system(f'python {evaluation_workspace}/KITTI_Tracking_Tools/align_trajectory.py --gt_traj_path={evaluation_workspace}/ground_truth_object/pose_gt_{seq_id}_{int(gt_trackid)}.txt --est_traj_path={evaluation_workspace}/results_object/pose_est_{seq_id}_{int(est_trackid)}.txt {seq_id} --gt_track_id={int(gt_trackid)} --est_track_id={int(est_trackid)} --gt_output_path={evaluation_workspace}/results_object/pose_gt_{seq_id}_{int(gt_trackid)}_aligned.txt --est_output_path={evaluation_workspace}/results_object/pose_est_{seq_id}_{int(est_trackid)}_aligned.txt')
                os.system(f'python ./Evaluation/KITTI_Tools/evaluate_kitti.py --gt_file={evaluation_workspace}/results_object/pose_gt_{seq_id}_{int(gt_trackid)}_aligned.txt  --est_file={evaluation_workspace}/results_object/pose_est_{seq_id}_{int(est_trackid)}_aligned.txt --output_dir={evaluation_workspace}/results_object/ --align=6dof --seq_id={i}_{int(gt_trackid)}')

                #use EVO tools to evaluate
                # os.system(f'evo_ape kitti {evaluation_workspace}/results_object/pose_gt_{seq_id}_{int(gt_trackid)}_aligned.txt {evaluation_workspace}/results_object/pose_est_{seq_id}_{int(est_trackid)}_aligned.txt  -a -v --no_warnings   -r trans_part --plot_mode=xz --save_plot {evaluation_workspace}/results_object/{seq_id}_{int(gt_trackid)}_ape --save_results {evaluation_workspace}/results_object/{seq_id}_{int(gt_trackid)}_ape.zip')
                # os.system(f'evo_rpe kitti {evaluation_workspace}/results_object/pose_gt_{seq_id}_{int(gt_trackid)}_aligned.txt {evaluation_workspace}/results_object/pose_est_{seq_id}_{int(est_trackid)}_aligned.txt -a -v --no_warnings  -r trans_part  --save_plot {evaluation_workspace}/results_object/{seq_id}_{int(gt_trackid)}_rpe_t --save_results {evaluation_workspace}/results_object/{seq_id}_{int(gt_trackid)}_rpe_t.zip')
                # os.system(f'evo_rpe kitti {evaluation_workspace}/results_object/pose_gt_{seq_id}_{int(gt_trackid)}_aligned.txt {evaluation_workspace}/results_object/pose_est_{seq_id}_{int(est_trackid)}_aligned.txt -a -v --no_warnings  -r angle_deg   --save_plot {evaluation_workspace}/results_object/{seq_id}_{int(gt_trackid)}_rpe_R --save_results {evaluation_workspace}/results_object/{seq_id}_{int(gt_trackid)}_rpe_R.zip')
                os.system(f'evo_traj kitti {evaluation_workspace}/results_object/pose_est_{seq_id}_{int(est_trackid)}_aligned.txt --ref={evaluation_workspace}/results_object/pose_gt_{seq_id}_{int(gt_trackid)}_aligned.txt -a --no_warnings    --plot_mode=xz  --save_plot {evaluation_workspace}/results_object/{seq_id}_{int(gt_trackid)}_traj') 
                # os.system(f'unzip -q -o {evaluation_workspace}/results_object/{seq_id}_{int(gt_trackid)}_ape.zip -d {evaluation_workspace}/results_object/{seq_id}_{int(gt_trackid)}_ape')
                # os.system(f'unzip -q -o {evaluation_workspace}/results_object/{seq_id}_{int(gt_trackid)}_rpe_t.zip -d {evaluation_workspace}/results_object/{seq_id}_{int(gt_trackid)}_rpe_t')
                # os.system(f'unzip -q -o {evaluation_workspace}/results_object/{seq_id}_{int(gt_trackid)}_rpe_R.zip -d {evaluation_workspace}/results_object/{seq_id}_{int(gt_trackid)}_rpe_R')