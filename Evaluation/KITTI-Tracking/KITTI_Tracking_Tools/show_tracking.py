import os
import cv2 as cv
import argparse
import numpy as np
import numpy.matlib 


def skew(phi):
    W = np.mat([[.0,-phi[2],phi[1]],[phi[2],0.0,-phi[0]],[-phi[1],phi[0],0.0]])
    W.reshape(3,3)
    return W

parser = argparse.ArgumentParser()
parser.add_argument('seq_path', type=str)
# parser.add_argument('seq_id', type=str, default=None)
# parser.add_argument('track_id', type=int, default=None)
# parser.add_argument('output_path', type=str, default="./")
args = parser.parse_args()

seq_path = args.seq_path
# seq_id = args.seq_id
# track_id_output = args.track_id
images_path = f'{seq_path}/image_2/'
tracklabels = open(f'{seq_path}/label.txt').readlines()
tracklabels = [line.split() for line in tracklabels]
calib = open(f'{seq_path}/calib.txt').readlines()
calib = [line.split() for line in calib]
cam_poses_gt = open(f'{seq_path}/pose_gt.txt').readlines()
cam_poses_gt = [[float(temp) for temp in line.split()] for line in cam_poses_gt]


Cam2Para = [float(value) for value in calib[2][1:]]
Kcam2 = np.mat([Cam2Para[0:3],Cam2Para[4:7],[0.0,0.0,1.0]])
Kcam2.reshape(3,3)

print("Camera para = ")
print(Kcam2)

Tcv = [float(value) for value in calib[5][1:]]
Tvi = [float(value) for value in calib[6][1:]]

frame_num = len(os.listdir(images_path)) 
tracklets = {}
for i in range(0,frame_num):
    tracklets[i] = []



for label in tracklabels:
    frame_id = int(label[0])
    track_id = int(label[1])
    type_str = label[2]
    observe_angle = float(label[5]) #-pi...pi
    bbox2d = [float(temp) for temp in label[6:10]]#left top right bottom
    dim3d = [float(temp) for temp in label[10:13]]#h, w, l
    location3d = [float(temp) for temp in label[13:16]]#x y z in camera coordinate
    rotation_y = float(label[16])#rotation ry around y axis of camera coordinate [-pi...pi]
    tracklet = [track_id, type_str, observe_angle, bbox2d, dim3d, location3d, rotation_y]
    tracklets[frame_id].append(tracklet)

obj_poses = {}
for i in range(0, frame_num):
    im = cv.imread(f'{images_path}/{str(i).zfill(6)}.png',-1)
    #get camera pose
    Twc = cam_poses_gt[i]
    Twc.extend([.0,.0,.0,1.0])
    Twc = np.mat(Twc).reshape(4,4)
    Rwc = Twc[0:3,0:3]
    twc = Twc[0:3,3]
    
    cur_tracklets = tracklets[i]
    for tracklet in cur_tracklets:
        track_id = tracklet[0]
        
        if(track_id != -1):
            ###############################draw tracklets########################
            tco = tracklet[5]
            dim3d = tracklet[4]
            bbox2d = tracklet[3]
            bbox2d = [int(temp) for temp in bbox2d]
            rotation_y = tracklet[6]
            phi_y = [0.0, rotation_y, 0.0]
            Rco = np.matlib.identity(3,float)+skew(phi_y)*np.sin(rotation_y)/rotation_y+skew(phi_y).dot(skew(phi_y))*(1-np.cos(rotation_y))/(rotation_y*rotation_y)

            h = dim3d[0]
            w = dim3d[1]
            l = dim3d[2]

            bbox3d_corners_obj = [[l/2,.0,w/2],[l/2,.0,-w/2],[-l/2,.0,-w/2],[-l/2,.0,w/2],[l/2,-h,w/2],[l/2,-h,-w/2],[-l/2,-h,w/2],[-l/2,-h,-w/2]]
            bbox3d_corners_im = []
            for corner3d_obj in bbox3d_corners_obj:
                corner3d_cam = Rco*np.mat(corner3d_obj).T+np.mat(tco).T
                corner3d_norm = (corner3d_cam/corner3d_cam[2])
                corner2d = Kcam2*corner3d_norm
                corner_pixel = corner2d.tolist()[0:2]
                corner_pixel = [int(temp[0]) for temp in corner_pixel]
                bbox3d_corners_im.append(corner_pixel)
            
            cv.line(im, bbox3d_corners_im[0],bbox3d_corners_im[4], (0,0,255),2)
            cv.line(im, bbox3d_corners_im[0],bbox3d_corners_im[3], (0,0,255),2)
            cv.line(im, bbox3d_corners_im[0],bbox3d_corners_im[1], (0,0,255),2)
            cv.line(im, bbox3d_corners_im[2],bbox3d_corners_im[1], (0,0,255),2)
            cv.line(im, bbox3d_corners_im[2],bbox3d_corners_im[7], (0,0,255),2)
            cv.line(im, bbox3d_corners_im[2],bbox3d_corners_im[3], (0,0,255),2)
            cv.line(im, bbox3d_corners_im[1],bbox3d_corners_im[5], (0,0,255),2)
            cv.line(im, bbox3d_corners_im[7],bbox3d_corners_im[5], (0,0,255),2)
            cv.line(im, bbox3d_corners_im[4],bbox3d_corners_im[5], (0,0,255),2)
            cv.line(im, bbox3d_corners_im[4],bbox3d_corners_im[6], (0,0,255),2)
            cv.line(im, bbox3d_corners_im[6],bbox3d_corners_im[7], (0,0,255),2)
            cv.line(im, bbox3d_corners_im[6],bbox3d_corners_im[3], (0,0,255),2)

            cv.rectangle(im,bbox2d[0:2],bbox2d[2:4],(255,0,0),2)
            w_2d = bbox2d[2]-bbox2d[0]
            h_2d = bbox2d[3]-bbox2d[1]
            text_h = int(h_2d/5)
            text_w = int(w_2d/5)
            
            cv.rectangle(im,[bbox2d[0],bbox2d[1]-text_h],[bbox2d[0]+text_w,bbox2d[1]],(255,0,0),-1)
            cv.putText(im,f'{track_id}',[bbox2d[0],bbox2d[1]], cv.FONT_HERSHEY_SIMPLEX, text_h/25,(255,255,255),2,cv.LINE_AA)
    
    
    cv.rectangle(im,[0,0],[50,30],(255,0,0),-1)
    cv.putText(im,f'{i}',[0,20], cv.FONT_HERSHEY_SIMPLEX, 0.5 ,(255,255,255),2,cv.LINE_AA)
            


            ###############################get object poses########################
    #         Rwo = (Rwc*Rco).tolist()
    #         two = (Rwc*np.mat(tco).T+twc).tolist()
    #         Two = Rwo
    #         Two[0].extend(two[0])
    #         Two[1].extend(two[1])
    #         Two[2].extend(two[2])
    #         obj_frame_id_pose = []
    #         obj_frame_id_pose.append(i)
    #         for row in Two:
    #             for value in row:
    #                 obj_frame_id_pose.append(value)
    #         print(obj_frame_id_pose)

    #         if(not track_id in obj_poses):
    #             obj_poses[track_id] = []
    #         obj_poses[track_id].append(obj_frame_id_pose)
    cv.imshow("img",im)
    cv.waitKey(0)


################generate seqid_trackid obj poses#############
# obj_poses_file = open(f'{output_path}/{int(seq_id)}_{track_id_output}.txt',"w+")

# obj_traj = obj_poses[track_id_output]
# pose_num = len(obj_traj)
# for i in range(0, pose_num):
#     pose = obj_traj[i]
#     for value in pose:
#         obj_poses_file.write(str(value)+" ")
#     if(i!=pose_num-1):
#         obj_poses_file.write('\n')

# obj_poses_file.close()






















