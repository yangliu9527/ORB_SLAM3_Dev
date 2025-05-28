import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--specify_date", type=str, default=None)
parser.add_argument("--specify_seq_id", type=int, default=None)

######################## evaluation parameters########################
specify_date = parser.parse_args().specify_date
specify_seq_id = parser.parse_args().specify_seq_id
######################## dataset and workspace path########################
dataset_path = "/home/brain/DataSet/KITTI-Raw/"
evaluation_workspace = "./Evaluation/KITTI-Raw/"

# seq ids for all evlaution
# all_dates_ids={'2011_09_26':[15, 27, 28, 29, 32, 52, 70, 101], '2011_09_29':[4], '2011_09_30':[16], '2011_10_03':[42, 47]}
# all_dates_ids={'2011_09_26':[9,13,14,51,101], '2011_09_29':[4], '2011_10_03':[47]} #for paper experiments
# all_dates_ids={'2011_09_26':[27,28,52,70,101]} #for other road sequences
#all_dates_ids={'2011_09_26':[15,29,32],'2011_09_29':[4],'2011_09_30':[16],'2011_10_03':[42, 47]} #for repeat road sequences
all_dates_ids = {'2011_10_03':[47]}

# select seq ids
if (specify_date == None):
    dates_ids = all_dates_ids
else:
    dates_ids[specify_date] = all_dates_ids[specify_date]
    if(specify_seq_id!=None):
        dates_ids[date] = specify_seq_id
print(f'seq ids for evalaution: {dates_ids}')


# evaluation
for date in dates_ids:
    seq_ids = dates_ids[date]
    for seq_id_int in seq_ids:
        seq_id_str = str(seq_id_int).zfill(4)
        seq_name = f'{date}_drive_{seq_id_str}_sync'
        seq_path = f'{seq_name}/{date}/{seq_name}'
        date_id = f'{date}_{seq_id_str}'
        ####################### run the system #######################
        #os.system(f'./Examples_added/Stereo/stereo_kitti_raw Vocabulary/ORBvoc.txt  Config/KITTI-Raw/color_23_exp/KITTI_{date_id}.yaml {dataset_path}/{seq_path}/')
        os.system(f'cp CameraTrajectory.txt {evaluation_workspace}/results/est_{date_id}.txt')
        os.system(f'python ./Evaluation/KITTI_Tools/evaluate_kitti.py --gt_file={evaluation_workspace}/ground_truth/pose_gt_{date_id}.txt  --est_file={evaluation_workspace}/results/est_{date_id}.txt --output_dir={evaluation_workspace}/results/ --align=6dof --seq_id={date_id}')
