# 分别提取真值和估计值中重合帧物体位姿，并将Tow转化为To1ok
import os
import numpy as np
import argparse

def generate_est_poses(Twes, start_id, end_id, Teg):
    traj_new = []
    for Twe in Twes:
        frame_id = int(Twe[0])
        Twe = Twe[1:]
        Twe.extend([.0,.0,.0,1.0])
        Twe = np.mat(Twe).reshape(4,4)
        Twg = Twe*Teg
        if(frame_id>=start_id and frame_id<=end_id):
            Twg = Twg[0:3][0:4].reshape(1,12).tolist() 
            Twg = [temp for temp in Twg][0]
            traj_new.append(Twg)
    return traj_new

def generate_gt_poses(Twgs, start_id, end_id):
    traj_new = []
    for Twg in Twgs:
        frame_id = int(Twg[0])
        Twg = Twg[1:]
        Twg.extend([.0,.0,.0,1.0])
        Twg = np.mat(Twg).reshape(4,4)
        if(frame_id>=start_id and frame_id<=end_id):
            Twg = Twg[0:3][0:4].reshape(1,12).tolist() 
            Twg = [temp for temp in Twg][0]
            traj_new.append(Twg)
    return traj_new

def write_txt(f, data):
    num = len(data)
    for i in range(0,num):
        line = data[i]
        num2 = len(line)
        for j in range(0,num2):
            value = line[j]
            if(j!=num2-1):
                f.write(str(value)+" ")
            else:
                f.write(str(value))
        if(i!=num-1):
            f.write("\n")
    f.close()


############################## 读取参数 ###############################
parser = argparse.ArgumentParser()
parser.add_argument('--gt_traj_path', type=str, default="./")
parser.add_argument('--est_traj_path', type=str, default="./")
parser.add_argument('seq_id', type=str, default=None)
parser.add_argument('--gt_track_id', type=str, default=None)
parser.add_argument('--est_track_id', type=str, default=None)
parser.add_argument('--gt_output_path', type=str, default="./")
parser.add_argument('--est_output_path', type=str, default="./")

args = parser.parse_args()
gt_output_path = args.gt_output_path
est_output_path = args.est_output_path
seq_id = args.seq_id
gt_track_id = args.gt_track_id
est_track_id = args.est_track_id

gt_traj = open(args.gt_traj_path).readlines()
gt_traj = [[float(temp) for temp in line.split()] for line in gt_traj]
est_traj = open(args.est_traj_path).readlines()
est_traj = [[float(temp) for temp in line.split()] for line in est_traj]

######################### 提取重合帧物体位姿To1ok ##########################
start_id = int(max(gt_traj[0][0],est_traj[0][0]))
end_id = int(min(gt_traj[-1][0],est_traj[-1][0]))

print(f'start from {start_id}th frame')
print(f'end at {end_id}th frame')
print(f'{end_id-start_id+1} poses for evaluation')


Twg0 = gt_traj[start_id-int(gt_traj[0][0])][1:]
Twg0.extend([.0,.0,.0,1.0])
Twg0 = np.mat(Twg0).reshape(4,4)

Twe0 = est_traj[start_id-int(est_traj[0][0])][1:]
Twe0.extend([.0,.0,.0,1.0])
Twe0 = np.mat(Twe0).reshape(4,4)

Teg = np.linalg.inv(Twe0)*Twg0

Twes_eva = generate_est_poses(est_traj, start_id, end_id, Teg)
Twgs_eva = generate_gt_poses(gt_traj, start_id, end_id)

################################## 写入文件 ##############################
gt_traj_new_file = open(f'{gt_output_path}', "w")
est_traj_new_file  = open(f'{est_output_path}',"w")
write_txt(gt_traj_new_file, Twgs_eva)
write_txt(est_traj_new_file, Twes_eva)

    







