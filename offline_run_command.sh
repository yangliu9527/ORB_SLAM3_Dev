#=========================KITTI=================#
#KITTI Stereo Inertial
./Examples_added/Stereo-Inertial/stereo_inertial_kitti_added Vocabulary/ORBvoc.txt Examples_added/Stereo-Inertial/KITTI00-02.yaml /home/brain/DataSet/KITTI-Odometry-Full/00
#KITTI Stereo 
./Examples_added/Stereo/stereo_kitti_added Vocabulary/ORBvoc.txt Examples_added/Stereo/KITTI00-02.yaml /home/brain/DataSet/KITTI-Odometry-Full/00
#KITTI Mono Inertial 
./Examples_added/Monocular-Inertial/mono_inertial_kitti_added Vocabulary/ORBvoc.txt Examples_added/Monocular-Inertial/KITTI00-02.yaml /home/brain/DataSet/KITTI-Odometry-Full/00
#KITTI Monocular
./Examples_added/Monocular/mono_kitti_added Vocabulary/ORBvoc.txt Examples_added/Monocular/KITTI00-02.yaml /home/brain/DataSet/KITTI-Odometry-Full/00

#=========================EuRoC=================#
#最后一个序列名称可以为空
#EuRoC Stereo Inertial
./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml /home/brain/DataSet/EuRoC/MH01 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereoi
#EuRoC Stereo 
./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml /home/brain/DataSet/EuRoC/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo
#EuRoC Mono Inertial 
./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml /home/brain/DataSet/EuRoC/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi
#EuRoC Monocular
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml /home/brain/DataSet/EuRoC/MH01 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono

#==============TUM VI==============#
#最后一个序列名称可以为空
#TUM VI Stereo Inertial
./Examples/Stereo-Inertial/stereo_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM-VI.yaml /home/brain/DataSet/TUM-VI/dataset-outdoors1_512_16/mav0/cam0/data /home/brain/DataSet/TUM-VI/dataset-outdoors1_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-outdoors1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-outdoors1_512.txt dataset-outdoors1_512_stereoi
#TUM VI Stereo 
./Examples/Stereo/stereo_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM-VI.yaml /home/brain/DataSet/TUM-VI/dataset-outdoors1_512_16/mav0/cam0/data /home/brain/DataSet/TUM-VI/dataset-outdoors1_512_16/mav0/cam1/data Examples/Stereo/TUM_TimeStamps/dataset-outdoors1_512.txt dataset-outdoors1_512_stereo
#TUM VI Mono Inertial 
./Examples/Monocular-Inertial/mono_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/TUM-VI.yaml /home/brain/DataSet/TUM-VI/dataset-outdoors1_512_16/mav0/cam0/data Examples/Monocular-Inertial/TUM_TimeStamps/dataset-outdoors1_512.txt Examples/Monocular-Inertial/TUM_IMU/dataset-outdoors1_512.txt dataset-outdoors1_512_monoi
#TUM VI Monocular
./Examples/Monocular/mono_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular/TUM-VI.yaml /home/brain/DataSet/TUM-VI/dataset-outdoors1_512_16/mav0/cam0/data Examples/Monocular/TUM_TimeStamps/dataset-outdoors1_512.txt dataset-outdoors1_512_mono

#Monocular Inertial multisession
./Examples/Monocular-Inertial/mono_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/TUM-VI.yaml /home/brain/DataSet/TUM-VI/dataset-outdoors1_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-outdoors1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-outdoors1_512.txt /home/brain/DataSet/TUM-VI/dataset-room2_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room2_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room2_512.txt /home/brain/DataSet/TUM-VI/dataset-room3_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room3_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room3_512.txt /home/brain/DataSet/TUM-VI/dataset-room4_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room4_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room4_512.txt /home/brain/DataSet/TUM-VI/dataset-room5_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room5_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room5_512.txt /home/brain/DataSet/TUM-VI/dataset-room6_512_16/mav0/cam0/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room6_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-room6_512.txt dataset-rooms123456_monoi

#Stereo Inertial multisession
./Examples/Stereo-Inertial/stereo_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM-VI.yaml /home/brain/DataSet/TUM-VI/dataset-outdoors1_512_16/mav0/cam0/data /home/brain/DataSet/TUM-VI/dataset-outdoors1_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-outdoors1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-outdoors1_512.txt /home/brain/DataSet/TUM-VI/dataset-magistrale1_512_16/mav0/cam0/data /home/brain/DataSet/TUM-VI/dataset-magistrale1_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-magistrale1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-magistrale1_512.txt /home/brain/DataSet/TUM-VI/dataset-magistrale5_512_16/mav0/cam0/data /home/brain/DataSet/TUM-VI/dataset-magistrale5_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-magistrale5_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-magistrale5_512.txt /home/brain/DataSet/TUM-VI/dataset-slides1_512_16/mav0/cam0/data /home/brain/DataSet/TUM-VI/dataset-slides1_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-slides1_512.txt Examples/Stereo-Inertial/TUM_IMU/dataset-slides1_512.txt dataset-room1_mag1_mag5_slides1_stereoi



#==============RGB-D======================#
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt /home/brain/DataSet/TUM/rgbd_dataset_freiburg2_desk/*.yaml  /home/brain/DataSet/TUM/rgbd_dataset_freiburg2_desk /home/brain/DataSet/TUM/rgbd_dataset_freiburg2_desk/associate.txt
