import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--plot",type=int,default=1)
plot_flag = parser.parse_args().plot

dataset_path = "/home/zhiyu/DataSet/EuRoC/"
evaluation_workspace = "./Evaluation/EuRoC/"
# seq_names = ['MH01', 'MH02', 'MH03',  'MH04','MH05','V101','V102','V103','V201','V202','V203']
seq_names = ['MH01']
for i in range(0,len(seq_names)):
    seq_name = seq_names[i]
    #run the system
    print(f"#######################evaluating {seq_name} seq camera motion#########################")
    os.system(f'./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml {dataset_path}/{seq_name} Config/EuRoC/EuRoC_TimeStamps/{seq_name}.txt') 
    os.system(f'cp CameraTrajectory.txt {evaluation_workspace}/results/CameraTrajectory_{seq_name}.txt')
    print(f"------------evaluating {seq_name} seq ATE------------")
    os.system(f'python2 ./Evaluation/TUM_Tools/evaluate_ate.py {evaluation_workspace}/ground_truth/{seq_name}_GT.txt {evaluation_workspace}/results/CameraTrajectory_{seq_name}.txt  --verbose --plot {evaluation_workspace}/results/{seq_name}.png --save_file {evaluation_workspace}/results/eva_{seq_name}.txt')
    print(f"------------evaluating {seq_name} seq RPE------------")
    os.system(f'python2 ./Evaluation/TUM_Tools/evaluate_rpe.py {evaluation_workspace}/ground_truth/{seq_name}_GT.txt {evaluation_workspace}/results/CameraTrajectory_{seq_name}.txt  --delta_unit=f --fixed_delta --verbose --plot {evaluation_workspace}/results/{seq_name}_rpe.png --save_file {evaluation_workspace}/results/{seq_name}_result_rpe.txt')    
   


