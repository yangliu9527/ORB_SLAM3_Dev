# 提取指定id物体的位姿Two
# 输入文件：KITTI-Tracking-Eva-new/est-obj/pose_est_{seq_id}.txt  格式：帧id + 物体类别 + 物体估计id + 物体位姿
# 输出文件：'KITTI-Tracking-Eva-new/est-obj/pose_est_{seq_id}_{track_id_output}.txt'  格式：frame id + 物体位姿

import os
import numpy as np
import argparse

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
parser.add_argument('est_objtraj_path', type=str)
parser.add_argument('seq_id', type=str, default=None)
parser.add_argument('track_id', type=int, default=None)
parser.add_argument('output_path', type=str, default="./est-obj/")
args = parser.parse_args()

output_path = args.output_path
est_objtraj_path = args.est_objtraj_path
track_id_output = args.track_id 
seq_id = args.seq_id


est_objtraj = open(est_objtraj_path).readlines()
est_objtraj = [[float(temp) for temp in line.split()] for line in est_objtraj]


######################## 提取指定id物体的位姿 ############################
est_objtraj_output = []
for tracklet in est_objtraj:
    frame_id = int(tracklet[0])
    track_id = int(tracklet[2])
    pose = tracklet[3:15]
    if(track_id == track_id_output):
        fid_pose = []
        fid_pose.append(frame_id)
        fid_pose.extend(pose)
        est_objtraj_output.append(fid_pose)

################################## 写入文件 ##############################
output_file = open(f'{output_path}',"w+")
write_txt(output_file, est_objtraj_output)
output_file.close()






