# python3 ./Eva/run_exp_tracking_OMD.py --cam_pose=1 --obj_pose=0 --mot=0 --plot=0 --f50=1
import os
import argparse
import re

parser = argparse.ArgumentParser()
parser.add_argument("--cam_pose",type=int,default=1)
parser.add_argument("--plot",type=int,default=1)
parser.add_argument("--seq_id",type=str,default="rgbd_dataset_freiburg2_desk")
parser.add_argument("--frame_num",type=int,default=500)

########################evaluation parameters########################
eva_campose = parser.parse_args().cam_pose
plot_flag = parser.parse_args().plot
seq_id = parser.parse_args().seq_id
frame_num = parser.parse_args().frame_num

print(f'evaluate cam_pose:{eva_campose}, plot figures:{plot_flag}')

########################dataset and workspace path########################
dataset_path = "/home/zhiyu/DataSet/TUM-RGBD/"


seq_ids =["rgbd_dataset_freiburg3_walking_xyz", "rgbd_dataset_freiburg2_desk"]

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
    # #####copy data to save
    # os.system(f'cp ObjectTrajectory.txt {evaluation_workspace}/results_object/pose_est_{frame_num}_{seq_id}.txt')
    # os.system(f'cp CameraTrajectory.txt {evaluation_workspace}/results_ego/pose_est_{frame_num}_{seq_id}.txt')

    # if(eva_campose):
    #     print(f"#############################evaluating camera pose#############################")
    #     os.system(f'python ./Evaluation/KITTI_Tools/evaluate_kitti.py --gt_file={evaluation_workspace}/ground_truth_ego/pose_gt_{frame_num}_{seq_id}.txt  --est_file={evaluation_workspace}/results_ego/pose_est_{frame_num}_{seq_id}.txt --output_dir={evaluation_workspace}/results_ego/ --align=6dof --seq_id=cam_{frame_num}_{seq_id}')

    # if(eva_objpose):
    #     relations = obj_gt_est_relation[seq_id]
    #     for relation in relations:
    #         gt_trackid = relation[0]
    #         est_trackid = relation[1]
    #         #######################object pose evaluation#######################
    #         if(eva_objpose):
    #             print(f"=======================================evaluating object {gt_trackid} pose=======================================")
    #             print(f"=======================================evaluating object {gt_trackid} pose=======================================")
    #             os.system(f'python {evaluation_workspace}/OMD_Tools/extract_objtraj.py {evaluation_workspace}/results_object/pose_est_{frame_num}_{seq_id}.txt {seq_id} {est_trackid} {evaluation_workspace}/results_object/pose_est_{frame_num}_{seq_id}_{est_trackid}.txt')
    #             os.system(f'python {evaluation_workspace}/OMD_Tools/align_trajectory.py --gt_traj_path={evaluation_workspace}/ground_truth_object/pose_gt_{frame_num}_{seq_id}_{int(gt_trackid)}.txt --est_traj_path={evaluation_workspace}/results_object/pose_est_{frame_num}_{seq_id}_{int(est_trackid)}.txt {seq_id} --gt_track_id={int(gt_trackid)} --est_track_id={int(est_trackid)} --gt_output_path={evaluation_workspace}/results_object/pose_gt_{frame_num}_{seq_id}_{int(gt_trackid)}_aligned.txt --est_output_path={evaluation_workspace}/results_object/pose_est_{frame_num}_{seq_id}_{int(est_trackid)}_aligned.txt')
    #             os.system(f'python ./Evaluation/KITTI_Tools/evaluate_kitti.py --gt_file={evaluation_workspace}/results_object/pose_gt_{frame_num}_{seq_id}_{int(gt_trackid)}_aligned.txt  --est_file={evaluation_workspace}/results_object/pose_est_{frame_num}_{seq_id}_{int(est_trackid)}_aligned.txt --output_dir={evaluation_workspace}/results_object/ --align=6dof --seq_id={frame_num}_{seq_id}_{int(gt_trackid)}')
    #             #os.system(f'python ./Evaluation/KITTI_Tools/evaluate_kitti.py --gt_file={evaluation_workspace}/results_object/pose_gt_{frame_num}_{seq_id}_{int(gt_trackid)}_aligned.txt  --est_file={evaluation_workspace}/results_object/pose_est_{frame_num}_{seq_id}_{int(est_trackid)}_aligned.txt --output_dir={evaluation_workspace}/results_object/ --seq_id={frame_num}_{seq_id}_{int(gt_trackid)}')