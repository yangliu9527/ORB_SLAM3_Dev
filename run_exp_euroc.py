'''
Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
Date: 2023-03-28 11:20:00
LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
LastEditTime: 2023-07-11 16:08:41
FilePath: /ORB_SLAM3_Line/run_exp_euroc.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''

import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--plot",type=int,default=1)
plot_flag = parser.parse_args().plot

dataset_path = "/home/zhiyu/DataSet/EuRoC/"
evaluation_workspace = "./eval_euroc"
# seq_names = ['MH01', 'MH02', 'MH03',  'MH04','MH05','V101','V102','V103','V201','V202','V203']
seq_names = ['V101','V102']
for i in range(0,len(seq_names)):
    seq_name = seq_names[i]
    #run the system
    print(f"#######################evaluating {seq_name} seq camera motion#########################")
    os.system(f'./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml {dataset_path}/{seq_name} Examples/Stereo/EuRoC_TimeStamps/{seq_name}.txt') 
    print(f"#######################evaluating {seq_name} seq camera motion#########################")
    os.system(f'cp CameraTrajectory.txt {evaluation_workspace}/eval_results/CameraTrajectory_{seq_name}.txt')
    os.system(f'python2 {evaluation_workspace}/evaluate_ate_scale.py {evaluation_workspace}/groundtruth/{seq_name}_GT.txt {evaluation_workspace}/eval_results/CameraTrajectory_{seq_name}.txt  --verbose --plot {evaluation_workspace}/eval_results/{seq_name}.png --save_file {evaluation_workspace}/eval_results/{seq_name}_result.txt')
        
   


