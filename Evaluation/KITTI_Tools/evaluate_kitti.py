# Copyright (C) Huangying Zhan 2019. All rights reserved.

import copy
from matplotlib import pyplot as plt
import matplotlib.pyplot as plt
# choose other backend that not required GUI (Agg, Cairo, PS, PDF or SVG) when use matplotlib
plt.switch_backend('agg')
import matplotlib.backends.backend_pdf
import tools.transformations as tr
import numpy as np
import os
from glob import glob
import argparse


def scale_lse_solver(X, Y):
    """Least-sqaure-error solver
    Compute optimal scaling factor so that s(X)-Y is minimum
    Args:
        X (KxN array): current data
        Y (KxN array): reference data
    Returns:
        scale (float): scaling factor
    """
    scale = np.sum(X * Y)/np.sum(X ** 2)
    return scale


def umeyama_alignment(x, y, with_scale=False):
    """
    Computes the least squares solution parameters of an Sim(m) matrix
    that minimizes the distance between a set of registered points.
    Umeyama, Shinji: Least-squares estimation of transformation parameters
                     between two point patterns. IEEE PAMI, 1991
    :param x: mxn matrix of points, m = dimension, n = nr. of data points
    :param y: mxn matrix of points, m = dimension, n = nr. of data points
    :param with_scale: set to True to align also the scale (default: 1.0 scale)
    :return: r, t, c - rotation matrix, translation vector and scale factor
    """
    if x.shape != y.shape:
        assert False, "x.shape not equal to y.shape"

    # m = dimension, n = nr. of data points
    m, n = x.shape

    # means, eq. 34 and 35
    mean_x = x.mean(axis=1)
    mean_y = y.mean(axis=1)

    # variance, eq. 36
    # "transpose" for column subtraction
    sigma_x = 1.0 / n * (np.linalg.norm(x - mean_x[:, np.newaxis])**2)

    # covariance matrix, eq. 38
    outer_sum = np.zeros((m, m))
    for i in range(n):
        outer_sum += np.outer((y[:, i] - mean_y), (x[:, i] - mean_x))
    cov_xy = np.multiply(1.0 / n, outer_sum)

    # SVD (text betw. eq. 38 and 39)
    u, d, v = np.linalg.svd(cov_xy)

    # S matrix, eq. 43
    s = np.eye(m)
    if np.linalg.det(u) * np.linalg.det(v) < 0.0:
        # Ensure a RHS coordinate system (Kabsch algorithm).
        s[m - 1, m - 1] = -1

    # rotation, eq. 40
    r = u.dot(s).dot(v)

    # scale & translation, eq. 42 and 41
    c = 1 / sigma_x * np.trace(np.diag(d).dot(s)) if with_scale else 1.0
    t = mean_y - np.multiply(c, r.dot(mean_x))

    return r, t, c


class KittiEvalOdom():
    """Evaluate odometry result
    Usage example:
        vo_eval = KittiEvalOdom()
        vo_eval.eval(gt_pose_txt_dir, result_pose_txt_dir)
    """

    def __init__(self):
        self.lengths = [100, 200, 300, 400, 500, 600, 700, 800]
        self.num_lengths = len(self.lengths)

    def load_poses_from_txt(self, file_name):
        """Load poses from txt (KITTI format)
        Each line in the file should follow one of the following structures
            (1) idx pose(3x4 matrix in terms of 12 numbers)
            (2) pose(3x4 matrix in terms of 12 numbers)

        Args:
            file_name (str): txt file path
        Returns:
            poses (dict): {idx: 4x4 array}
        """
        f = open(file_name, 'r')
        s = f.readlines()
        f.close()
        poses = {}
        for cnt, line in enumerate(s):
            P = np.eye(4)
            line_split = [float(i) for i in line.split(" ") if i != ""]
            withIdx = len(line_split) == 13
            for row in range(3):
                for col in range(4):
                    P[row, col] = line_split[row*4 + col + withIdx]
            if withIdx:
                frame_idx = line_split[0]
            else:
                frame_idx = cnt
            poses[frame_idx] = P
        return poses

    def trajectory_distances(self, poses):
        """Compute distance for each pose w.r.t frame-0
        Args:
            poses (dict): {idx: 4x4 array}
        Returns:
            dist (float list): distance of each pose w.r.t frame-0
        """
        dist = [0]
        sort_frame_idx = sorted(poses.keys())
        for i in range(len(sort_frame_idx)-1):
            cur_frame_idx = sort_frame_idx[i]
            next_frame_idx = sort_frame_idx[i+1]
            P1 = poses[cur_frame_idx]
            P2 = poses[next_frame_idx]
            dx = P1[0, 3] - P2[0, 3]
            dy = P1[1, 3] - P2[1, 3]
            dz = P1[2, 3] - P2[2, 3]
            dist.append(dist[i]+np.sqrt(dx**2+dy**2+dz**2))
        return dist

    def rotation_error(self, pose_error):
        """Compute rotation error
        Args:
            pose_error (4x4 array): relative pose error
        Returns:
            rot_error (float): rotation error
        """
        a = pose_error[0, 0]
        b = pose_error[1, 1]
        c = pose_error[2, 2]
        d = 0.5*(a+b+c-1.0)
        rot_error = np.arccos(max(min(d, 1.0), -1.0))
        return rot_error

    def translation_error(self, pose_error):
        """Compute translation error
        Args:
            pose_error (4x4 array): relative pose error
        Returns:
            trans_error (float): translation error
        """
        dx = pose_error[0, 3]
        dy = pose_error[1, 3]
        dz = pose_error[2, 3]
        trans_error = np.sqrt(dx**2+dy**2+dz**2)
        return trans_error

    def last_frame_from_segment_length(self, dist, first_frame, length):
        """Find frame (index) that away from the first_frame with
        the required distance
        Args:
            dist (float list): distance of each pose w.r.t frame-0
            first_frame (int): start-frame index
            length (float): required distance
        Returns:
            i (int) / -1: end-frame index. if not found return -1
        """
        for i in range(first_frame, len(dist), 1):
            if dist[i] > (dist[first_frame] + length):
                return i
        return -1

    def calc_sequence_errors(self, poses_gt, poses_result):
        """calculate sequence error
        Args:
            poses_gt (dict): {idx: 4x4 array}, ground truth poses
            poses_result (dict): {idx: 4x4 array}, predicted poses
        Returns:
            err (list list): [first_frame, rotation error, translation error, length, speed]
                - first_frame: frist frame index
                - rotation error: rotation error per length
                - translation error: translation error per length
                - length: evaluation trajectory length
                - speed: car speed (#FIXME: 10FPS is assumed)
        """
        err = []
        dist = self.trajectory_distances(poses_gt)
        self.step_size = 10

        for first_frame in range(0, len(poses_gt), self.step_size):
            for i in range(self.num_lengths):
                len_ = self.lengths[i]
                last_frame = self.last_frame_from_segment_length(
                    dist, first_frame, len_
                )

                # Continue if sequence not long enough
                if last_frame == -1 or \
                        not (last_frame in poses_result.keys()) or \
                        not (first_frame in poses_result.keys()):
                    continue

                # compute rotational and translational errors
                pose_delta_gt = np.dot(
                    np.linalg.inv(poses_gt[first_frame]),
                    poses_gt[last_frame]
                )
                pose_delta_result = np.dot(
                    np.linalg.inv(poses_result[first_frame]),
                    poses_result[last_frame]
                )
                pose_error = np.dot(
                    np.linalg.inv(pose_delta_result),
                    pose_delta_gt
                )

                r_err = self.rotation_error(pose_error)
                t_err = self.translation_error(pose_error)

                # compute speed
                num_frames = last_frame - first_frame + 1.0
                speed = len_/(0.1*num_frames)

                err.append([first_frame, r_err/len_, t_err/len_, len_, speed])
        return err

    def save_sequence_errors(self, err, file_name):
        """Save sequence error
        Args:
            err (list list): error information
            file_name (str): txt file for writing errors
        """
        fp = open(file_name, 'w')
        for i in err:
            line_to_write = " ".join([str(j) for j in i])
            fp.writelines(line_to_write+"\n")
        fp.close()

    def compute_overall_err(self, seq_err):
        """Compute average translation & rotation errors
        Args:
            seq_err (list list): [[r_err, t_err],[r_err, t_err],...]
                - r_err (float): rotation error
                - t_err (float): translation error
        Returns:
            ave_t_err (float): average translation error
            ave_r_err (float): average rotation error
        """
        t_err = 0
        r_err = 0

        seq_len = len(seq_err)

        if seq_len > 0:
            for item in seq_err:
                r_err += item[1]
                t_err += item[2]
            ave_t_err = t_err / seq_len
            ave_r_err = r_err / seq_len
            return ave_t_err, ave_r_err
        else:
            return 0, 0

    def plot_trajectory(self, poses_gt, poses_result, dir_name):
        """Plot trajectory for both GT and prediction
        Args:
            poses_gt (dict): {idx: 4x4 array}; ground truth poses
            poses_result (dict): {idx: 4x4 array}; predicted poses
            seq (int): sequence index.
        """
        plot_keys = ["Ground Truth", "Ours"]
        fontsize_ = 20

        poses_dict = {}
        poses_dict["Ground Truth"] = poses_gt
        poses_dict["Ours"] = poses_result

        fig = plt.figure()
        ax = plt.gca()
        ax.set_aspect('equal')

        for key in plot_keys:
            pos_xz = []
            frame_idx_list = sorted(poses_dict["Ours"].keys())
            for frame_idx in frame_idx_list:
                # pose = np.linalg.inv(poses_dict[key][frame_idx_list[0]]) @ poses_dict[key][frame_idx]
                pose = poses_dict[key][frame_idx]
                pos_xz.append([pose[0, 3],  pose[2, 3]])
            pos_xz = np.asarray(pos_xz)
            plt.plot(pos_xz[:, 0],  pos_xz[:, 1], label=key)

        plt.legend(loc="upper right", prop={'size': fontsize_})
        plt.xticks(fontsize=fontsize_)
        plt.yticks(fontsize=fontsize_)
        plt.xlabel('x (m)', fontsize=fontsize_)
        plt.ylabel('z (m)', fontsize=fontsize_)
        fig.set_size_inches(10, 10)
        #png_title = "sequence_{:02}".format(seq)
        #fig_pdf = dir_name + "_trajectory.pdf"
        fig_png = dir_name + "_trajectory.png"
        plt.savefig(fig_png, bbox_inches='tight', pad_inches=0)
        plt.close(fig)
    
    def plot_RPE(self, poses_ref, poses_pred, plot_path_dir):
        trans_errors = []
        rot_errors = []
        for i in list(poses_pred.keys())[:-1]:
            gt1 = poses_ref[i]
            gt2 = poses_ref[i+1]
            gt_rel = np.linalg.inv(gt1) @ gt2

            pred1 = poses_pred[i]
            pred2 = poses_pred[i+1]
            pred_rel = np.linalg.inv(pred1) @ pred2
            rel_err = np.linalg.inv(gt_rel) @ pred_rel

            trans_errors.append(self.translation_error(rel_err))
            rot_errors.append(self.rotation_error(rel_err))
        # rpe_trans = np.sqrt(np.mean(np.asarray(trans_errors) ** 2))
        # rpe_rot = np.sqrt(np.mean(np.asarray(rot_errors) ** 2))
        # rpe_trans = np.mean(np.asarray(trans_errors))
        # rpe_rot = np.mean(np.asarray(rot_errors))

        plot_x = [ℹ for i in range(len(trans_errors))]
        fontsize_ = 10
        # draw RPE of translation
        fig_RPEt = plt.figure()
        plt.plot(plot_x, trans_errors, '-', label="Relative Translation Error")
        plt.ylabel('RPE of translation (m)', fontsize=fontsize_)
        plt.xlabel('index', fontsize=fontsize_)
        plt.legend(loc="upper right", prop={'size': fontsize_})
        RPEt_fig_png = plot_path_dir+"_RPEt.png"
        #plt.savefig(fig_pdf, bbox_inches='tight', pad_inches=0)
        plt.savefig(RPEt_fig_png, bbox_inches='tight', pad_inches=0)
        plt.close(fig_RPEt)
        # draw RPE of rotation

        fig_RPEr = plt.figure()
        plt.plot(plot_x, rot_errors, '-', label="Relative Rotation Error")
        plt.ylabel('RPE of rotation (deg)', fontsize=fontsize_)
        plt.xlabel('index', fontsize=fontsize_)
        plt.legend(loc="upper right", prop={'size': fontsize_})
        RPEr_fig_png = plot_path_dir+"_RPEr.png"
        #plt.savefig(fig_pdf, bbox_inches='tight', pad_inches=0)
        plt.savefig(RPEr_fig_png, bbox_inches='tight', pad_inches=0)
        plt.close(fig_RPEr)

    def plot_ATE(self, gt, pred, plot_path_dir):
        """Compute RMSE of ATE
        Args:
            gt (4x4 array dict): ground-truth poses
            pred (4x4 array dict): predicted poses
        """
        errors = []
        idx_0 = list(pred.keys())[0]
        gt_0 = gt[idx_0]
        pred_0 = pred[idx_0]

        for i in pred:
            # cur_gt = np.linalg.inv(gt_0) @ gt[i]
            cur_gt = gt[i]
            gt_xyz = cur_gt[:3, 3]

            # cur_pred = np.linalg.inv(pred_0) @ pred[i]
            cur_pred = pred[i]
            pred_xyz = cur_pred[:3, 3]

            align_err = gt_xyz - pred_xyz

            # print('i: ', i)
            # print("gt: ", gt_xyz)
            # print("pred: ", pred_xyz)
            # input("debug")
            errors.append(np.sqrt(np.sum(align_err ** 2)))
        ate = np.sqrt(np.mean(np.asarray(errors) ** 2))

        plot_x = [i for i in range(len(errors))]
        fontsize_ = 10
        # draw ATE of translation
        fig_ATE = plt.figure()
        plt.plot(plot_x, errors, '-', label="Absolute Translation Error")
        plt.ylabel('ATE of translation (m)', fontsize=fontsize_)
        plt.xlabel('index', fontsize=fontsize_)
        plt.legend(loc="upper right", prop={'size': fontsize_})
        ATE_fig_png = plot_path_dir+"_ATE.png"
        #plt.savefig(fig_pdf, bbox_inches='tight', pad_inches=0)
        plt.savefig(ATE_fig_png, bbox_inches='tight', pad_inches=0)
        plt.close(fig_ATE)
        

    def plot_xyz(self, poses_ref, poses_pred, plot_path_dir):
        
        def traj_xyz(axarr, positions_xyz, style='-', color='black', title="", label="", alpha=1.0):
            """
                plot a path/trajectory based on xyz coordinates into an axis
                :param axarr: an axis array (for x, y & z) e.g. from 'fig, axarr = plt.subplots(3)'
                :param traj: trajectory
                :param style: matplotlib line style
                :param color: matplotlib color
                :param label: label (for legend)
                :param alpha: alpha value for transparency
            """
            x = range(0, len(positions_xyz))
            xlabel = "index"
            ylabels = ["$x$ (m)", "$y$ (m)", "$z$ (m)"]
            # plt.title('PRY')
            for i in range(0, 3):
                axarr[i].plot(x, positions_xyz[:, i], style, color=color, label=label, alpha=alpha)
                axarr[i].set_ylabel(ylabels[i])
                axarr[i].legend(loc="upper right", frameon=True)
            axarr[2].set_xlabel(xlabel)
            if title:
                axarr[0].set_title('XYZ')           

        fig, axarr = plt.subplots(3, sharex="col", figsize=tuple([20, 10]))  
        
        pred_xyz = np.array([p[:3, 3] for _,p in poses_pred.items()])
        traj_xyz(axarr, pred_xyz, '-', 'b', title='XYZ', label='Ours', alpha=1.0)
        if poses_ref:
            ref_xyz = np.array([p[:3, 3] for _,p in poses_ref.items()])
            traj_xyz(axarr, ref_xyz, '-', 'r', label='GT', alpha=1.0)
      
        plt.savefig(plot_path_dir + "_traj_xyz.png", bbox_inches='tight', pad_inches=0.1)
        #pdf = matplotlib.backends.backend_pdf.PdfPages(plot_path_dir +  "/" + name + ".pdf")        
        fig.tight_layout()
        #pdf.savefig(fig)       
        # plt.show()
        #pdf.close()

    def plot_rpy(self, poses_ref, poses_pred, plot_path_dir, axes='szxy'):
        
        def traj_rpy(axarr, orientations_euler, style='-', color='black', title="", label="", alpha=1.0):
            """
            plot a path/trajectory's Euler RPY angles into an axis
            :param axarr: an axis array (for R, P & Y) e.g. from 'fig, axarr = plt.subplots(3)'
            :param traj: trajectory
            :param style: matplotlib line style
            :param color: matplotlib color
            :param label: label (for legend)
            :param alpha: alpha value for transparency
            """
            x = range(0, len(orientations_euler))
            xlabel = "index"
            ylabels = ["$roll$ (deg)", "$pitch$ (deg)", "$yaw$ (deg)"]
            # plt.title('PRY')
            for i in range(0, 3):
                axarr[i].plot(x, np.rad2deg(orientations_euler[:, i]), style,
                            color=color, label=label, alpha=alpha)
                axarr[i].set_ylabel(ylabels[i])
                axarr[i].legend(loc="upper right", frameon=True)
            axarr[2].set_xlabel(xlabel)
            if title:
                axarr[0].set_title('PRY')           

        fig_rpy, axarr_rpy = plt.subplots(3, sharex="col", figsize=tuple([20, 10]))

        pred_rpy = np.array([tr.euler_from_matrix(p, axes=axes) for _,p in poses_pred.items()])
        traj_rpy(axarr_rpy, pred_rpy, '-', 'b', title='RPY', label='Ours', alpha=1.0)
        if poses_ref:
            ref_rpy = np.array([tr.euler_from_matrix(p, axes=axes) for _,p in poses_ref.items()])
            traj_rpy(axarr_rpy, ref_rpy, '-', 'r', label='GT', alpha=1.0)

        plt.savefig(plot_path_dir + "_traj_rpy.png", bbox_inches='tight', pad_inches=0.1)
        #pdf = matplotlib.backends.backend_pdf.PdfPages(plot_path_dir +  "/" + name + ".pdf")        
        fig_rpy.tight_layout()
        #pdf.savefig(fig_rpy)       
        # plt.show()
        #pdf.close()

    def plotPath_2D_3(self, seq, poses_gt, poses_result, plot_path_dir):
        '''
            plot path in XY, XZ and YZ plane
        '''
        fontsize_ = 10
        plot_keys = ["Ground Truth", "Ours"]
        start_point = [0, 0]
        style_pred = 'b-'
        style_gt = 'r-'
        style_O = 'ko'

        ### get the value
        if poses_gt: 
            poses_gt = [(k,poses_gt[k]) for k in sorted(poses_gt.keys())]
            x_gt = np.asarray([pose[0,3] for _,pose in poses_gt])
            y_gt = np.asarray([pose[1,3] for _,pose in poses_gt])
            z_gt = np.asarray([pose[2,3] for _,pose in poses_gt])
        poses_result = [(k,poses_result[k]) for k in sorted(poses_result.keys())]
        x_pred = np.asarray([pose[0,3] for _,pose in poses_result])
        y_pred = np.asarray([pose[1,3] for _,pose in poses_result])
        z_pred = np.asarray([pose[2,3] for _,pose in poses_result])
        
        fig = plt.figure(figsize=(20,6), dpi=100)
        ### plot the figure
        plt.subplot(1,3,1)
        ax = plt.gca()
        if poses_gt: plt.plot(x_gt, z_gt, style_gt, label=plot_keys[0])
        plt.plot(x_pred, z_pred, style_pred, label=plot_keys[1])
        plt.plot(start_point[0], start_point[1], style_O, label='Start Point')
        plt.legend(loc="upper right", prop={'size':fontsize_})
        plt.xlabel('x (m)', fontsize=fontsize_)
        plt.ylabel('z (m)', fontsize=fontsize_)
        ### set the range of x and y
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        xmean = np.mean(xlim)
        ymean = np.mean(ylim)
        plot_radius = max([abs(lim - mean_)
                            for lims, mean_ in ((xlim, xmean),
                                                (ylim, ymean))
                            for lim in lims])
        ax.set_xlim([xmean - plot_radius, xmean + plot_radius])
        ax.set_ylim([ymean - plot_radius, ymean + plot_radius])

        plt.subplot(1,3,2)
        ax = plt.gca()
        if poses_gt: plt.plot(x_gt, y_gt, style_gt, label=plot_keys[0])
        plt.plot(x_pred, y_pred, style_pred, label=plot_keys[1])
        plt.plot(start_point[0], start_point[1], style_O, label='Start Point')
        plt.legend(loc="upper right", prop={'size':fontsize_})
        plt.xlabel('x (m)', fontsize=fontsize_)
        plt.ylabel('y (m)', fontsize=fontsize_)
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        xmean = np.mean(xlim)
        ymean = np.mean(ylim)
        ax.set_xlim([xmean - plot_radius, xmean + plot_radius])
        ax.set_ylim([ymean - plot_radius, ymean + plot_radius])

        plt.subplot(1,3,3)
        ax = plt.gca()
        if poses_gt: plt.plot(y_gt, z_gt, style_gt, label=plot_keys[0])
        plt.plot(y_pred, z_pred, style_pred, label=plot_keys[1])
        plt.plot(start_point[0], start_point[1], style_O, label='Start Point')
        plt.legend(loc="upper right", prop={'size':fontsize_})
        plt.xlabel('y (m)', fontsize=fontsize_)
        plt.ylabel('z (m)', fontsize=fontsize_)
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        xmean = np.mean(xlim)
        ymean = np.mean(ylim)
        ax.set_xlim([xmean - plot_radius, xmean + plot_radius])
        ax.set_ylim([ymean - plot_radius, ymean + plot_radius])

        png_title = "{}_path".format(seq)
        plt.savefig(plot_path_dir +  "/" + png_title + ".png", bbox_inches='tight', pad_inches=0.1)
        pdf = matplotlib.backends.backend_pdf.PdfPages(plot_path_dir +  "/" + png_title + ".pdf")        
        fig.tight_layout()
        pdf.savefig(fig)  
        # plt.show()
        plt.close()


    def plot_segment_error(self, avg_segment_errs, dir_name):
        """Plot per-length error
        Args:
            avg_segment_errs (dict): {100:[avg_t_err, avg_r_err],...}
            seq (int): sequence index.
        """
        # Translation error
        plot_y = []
        plot_x = []
        for len_ in self.lengths:
            plot_x.append(len_)
            if len(avg_segment_errs[len_]) > 0:
                plot_y.append(avg_segment_errs[len_][0] * 100)
            else:
                plot_y.append(0)
        fontsize_ = 10
        fig = plt.figure()
        plt.plot(plot_x, plot_y, "bs-", label="Translation Error")
        plt.ylabel('Translation Error (%)', fontsize=fontsize_)
        plt.xlabel('Path Length (m)', fontsize=fontsize_)
        plt.legend(loc="upper right", prop={'size': fontsize_})
        fig.set_size_inches(5, 5)
        fig_pdf = dir_name+"_trans_err.pdf"
        fig_png = dir_name+"_trans_err.png"
        #plt.savefig(fig_pdf, bbox_inches='tight', pad_inches=0)
        plt.savefig(fig_png, bbox_inches='tight', pad_inches=0)
        plt.close(fig)

        # Rotation error
        plot_y = []
        plot_x = []
        for len_ in self.lengths:
            plot_x.append(len_)
            if len(avg_segment_errs[len_]) > 0:
                plot_y.append(avg_segment_errs[len_][1] / np.pi * 180 * 100)
            else:
                plot_y.append(0)
        fontsize_ = 10
        fig = plt.figure()
        plt.plot(plot_x, plot_y, "bs-", label="Rotation Error")
        plt.ylabel('Rotation Error (deg/100m)', fontsize=fontsize_)
        plt.xlabel('Path Length (m)', fontsize=fontsize_)
        plt.legend(loc="upper right", prop={'size': fontsize_})
        fig.set_size_inches(5, 5)
        fig_pdf = dir_name+"_rot_err.pdf"
        fig_png = dir_name+"_rot_err.png"
        #plt.savefig(fig_pdf, bbox_inches='tight', pad_inches=0)
        plt.savefig(fig_png, bbox_inches='tight', pad_inches=0)
        plt.close(fig)

    def compute_segment_error(self, seq_errs):
        """This function calculates average errors for different segment.
        Args:
            seq_errs (list list): list of errs; [first_frame, rotation error, translation error, length, speed]
                - first_frame: frist frame index
                - rotation error: rotation error per length
                - translation error: translation error per length
                - length: evaluation trajectory length
                - speed: car speed (#FIXME: 10FPS is assumed)
        Returns:
            avg_segment_errs (dict): {100:[avg_t_err, avg_r_err],...}    
        """

        segment_errs = {}
        avg_segment_errs = {}
        for len_ in self.lengths:
            segment_errs[len_] = []

        # Get errors
        for err in seq_errs:
            len_ = err[3]
            t_err = err[2]
            r_err = err[1]
            segment_errs[len_].append([t_err, r_err])

        # Compute average
        for len_ in self.lengths:
            if segment_errs[len_] != []:
                avg_t_err = np.mean(np.asarray(segment_errs[len_])[:, 0])
                avg_r_err = np.mean(np.asarray(segment_errs[len_])[:, 1])
                avg_segment_errs[len_] = [avg_t_err, avg_r_err]
            else:
                avg_segment_errs[len_] = []
        return avg_segment_errs

    def compute_ATE(self, gt, pred):
        """Compute RMSE of ATE
        Args:
            gt (4x4 array dict): ground-truth poses
            pred (4x4 array dict): predicted poses
        """
        errors = []
        idx_0 = list(pred.keys())[0]
        gt_0 = gt[idx_0]
        pred_0 = pred[idx_0]

        for i in pred:
            # cur_gt = np.linalg.inv(gt_0) @ gt[i]
            cur_gt = gt[i]
            gt_xyz = cur_gt[:3, 3]

            # cur_pred = np.linalg.inv(pred_0) @ pred[i]
            cur_pred = pred[i]
            pred_xyz = cur_pred[:3, 3]

            align_err = gt_xyz - pred_xyz

            # print('i: ', i)
            # print("gt: ", gt_xyz)
            # print("pred: ", pred_xyz)
            # input("debug")
            errors.append(np.sqrt(np.sum(align_err ** 2)))
        ate = np.sqrt(np.mean(np.asarray(errors) ** 2))
        return ate

    def compute_RPE(self, gt, pred):
        """Compute RPE
        Args:
            gt (4x4 array dict): ground-truth poses
            pred (4x4 array dict): predicted poses
        Returns:
            rpe_trans
            rpe_rot
        """
        trans_errors = []
        rot_errors = []
        for i in list(pred.keys())[:-1]:
            gt1 = gt[i]
            gt2 = gt[i+1]
            gt_rel = np.linalg.inv(gt1) @ gt2

            pred1 = pred[i]
            pred2 = pred[i+1]
            pred_rel = np.linalg.inv(pred1) @ pred2
            rel_err = np.linalg.inv(gt_rel) @ pred_rel

            trans_errors.append(self.translation_error(rel_err))
            rot_errors.append(self.rotation_error(rel_err))
        rpe_trans = np.sqrt(np.mean(np.asarray(trans_errors) ** 2))
        rpe_rot = np.sqrt(np.mean(np.asarray(rot_errors) ** 2))
        # rpe_trans = np.mean(np.asarray(trans_errors))
        # rpe_rot = np.mean(np.asarray(rot_errors))
        return rpe_trans, rpe_rot

    def scale_optimization(self, gt, pred):
        """ Optimize scaling factor
        Args:
            gt (4x4 array dict): ground-truth poses
            pred (4x4 array dict): predicted poses
        Returns:
            new_pred (4x4 array dict): predicted poses after optimization
        """
        pred_updated = copy.deepcopy(pred)
        xyz_pred = []
        xyz_ref = []
        for i in pred:
            pose_pred = pred[i]
            pose_ref = gt[i]
            xyz_pred.append(pose_pred[:3, 3])
            xyz_ref.append(pose_ref[:3, 3])
        xyz_pred = np.asarray(xyz_pred)
        xyz_ref = np.asarray(xyz_ref)
        scale = scale_lse_solver(xyz_pred, xyz_ref)
        for i in pred_updated:
            pred_updated[i][:3, 3] *= scale
        return pred_updated

    def write_result(self, f, seq, errs):
        """Write result into a txt file
        Args:
            f (IOWrapper)
            seq (int): sequence number
            errs (list): [ave_t_err, ave_r_err, ate, rpe_trans, rpe_rot]
        """
        ave_t_err, ave_r_err, ate, rpe_trans, rpe_rot = errs
        lines = []
        lines.append("Sequence: \t {} \n".format(seq))
        lines.append("Trans. err. (%): \t {:.3f} \n".format(ave_t_err*100))
        lines.append(
            "Rot. err. (deg/100m): \t {:.3f} \n".format(ave_r_err/np.pi*180*100))
        lines.append("ATE (m): \t {:.3f} \n".format(ate))
        lines.append("RPE (m): \t {:.3f} \n".format(rpe_trans))
        lines.append("RPE (deg): \t {:.3f} \n\n".format(rpe_rot * 180 / np.pi))
        for line in lines:
            f.writelines(line)

    def eval_single(self, gt_file_dir, est_file_dir, result_dir, seq_id, alignment=None):

        #seq_id_str = str(seq_id).zfill(2)
        seq_id_str = seq_id
        err_output = result_dir+"/"+seq_id_str+"_seq_error.txt"
        eva_output = result_dir+"/"+seq_id_str+"_eva_result.txt"
        err_plot_output = result_dir+"/"+seq_id_str
        traj_plot_output = result_dir+"/"+seq_id_str
        f = open(eva_output, 'w+')

        ave_t_errs = []
        ave_r_errs = []
        seq_ate = []
        seq_rpe_trans = []
        seq_rpe_rot = []

        poses_est = self.load_poses_from_txt(est_file_dir)
        poses_gt = self.load_poses_from_txt(gt_file_dir)

        idx_0 = sorted(list(poses_est.keys()))[0]
        pred_0 = poses_est[idx_0]
        gt_0 = poses_gt[idx_0]
        for cnt in poses_est:
            poses_est[cnt] = np.linalg.inv(pred_0) @ poses_est[cnt]
            poses_gt[cnt] = np.linalg.inv(gt_0) @ poses_gt[cnt]

        if alignment == "scale":
            poses_est = self.scale_optimization(poses_gt, poses_est)
        elif alignment == "scale_7dof" or alignment == "7dof" or alignment == "6dof":
            # get XYZ
            xyz_gt = []
            xyz_result = []
            for cnt in poses_est:
                xyz_gt.append([poses_gt[cnt][0, 3], poses_gt[cnt]
                              [1, 3], poses_gt[cnt][2, 3]])
                xyz_result.append(
                    [poses_est[cnt][0, 3], poses_est[cnt][1, 3], poses_est[cnt][2, 3]])
            xyz_gt = np.asarray(xyz_gt).transpose(1, 0)
            xyz_result = np.asarray(xyz_result).transpose(1, 0)

            r, t, scale = umeyama_alignment(
                xyz_result, xyz_gt, alignment != "6dof")

            align_transformation = np.eye(4)
            align_transformation[:3:, :3] = r
            align_transformation[:3, 3] = t

            for cnt in poses_est:
                poses_est[cnt][:3, 3] *= scale
                if alignment == "7dof" or alignment == "6dof":
                    poses_est[cnt] = align_transformation @ poses_est[cnt]

        # compute sequence errors
        seq_err = self.calc_sequence_errors(poses_gt, poses_est)
        # self.save_sequence_errors(seq_err, err_output)

        # Compute segment errors
        avg_segment_errs = self.compute_segment_error(seq_err)

        # compute overall error
        ave_t_err, ave_r_err = self.compute_overall_err(seq_err)
        print("Sequence: " + seq_id_str)
        print("Translational error (%): ", ave_t_err*100)
        print("Rotational error (deg/100m): ", ave_r_err/np.pi*180*100)
        ave_t_errs.append(ave_t_err)
        ave_r_errs.append(ave_r_err)

        # Compute ATE
        ate = self.compute_ATE(poses_gt, poses_est)
        seq_ate.append(ate)
        print("ATE (m): ", ate)

        # Compute RPE
        rpe_trans, rpe_rot = self.compute_RPE(poses_gt, poses_est)
        seq_rpe_trans.append(rpe_trans)
        seq_rpe_rot.append(rpe_rot)
        print("RPE (m): ", rpe_trans)
        print("RPE (deg): ", rpe_rot * 180 / np.pi)

        # Plotting
        self.plot_trajectory(poses_gt, poses_est,traj_plot_output)
        self.plot_segment_error(avg_segment_errs, err_plot_output)
        self.plot_xyz(poses_gt, poses_est,traj_plot_output)
        self.plot_rpy(poses_gt, poses_est,traj_plot_output)
        self.plot_RPE(poses_gt, poses_est,traj_plot_output)
        self.plot_ATE(poses_gt, poses_est,traj_plot_output)

        # Save result summary
        self.write_result(
            f, seq_id, [ave_t_err, ave_r_err, ate, rpe_trans, rpe_rot])

        # print("-------------------- For Copying ------------------------------")
        # for i in range(len(ave_t_errs)):
        #     print("{0:.2f}".format(ave_t_errs[i]*100))
        #     print("{0:.2f}".format(ave_r_errs[i]/np.pi*180*100))
        #     print("{0:.2f}".format(seq_ate[i]))
        #     print("{0:.3f}".format(seq_rpe_trans[i]))
        #     print("{0:.3f}".format(seq_rpe_rot[i] * 180 / np.pi))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='KITTI evaluation')
    parser.add_argument('--gt_file', type=str, required=True,
                        help="ground truth file")
    parser.add_argument('--est_file', type=str, required=True,
                        help="estimation file")
    parser.add_argument('--output_dir', type=str, required=True,
                        help="output directory")
    parser.add_argument('--align', type=str,
                        choices=['scale', 'scale_7dof', '7dof', '6dof'],
                        default=None,
                        help="alignment type")
    parser.add_argument('--seq_id',
                        type=str,
                        help="sequence to be evaluated",
                        default=None)
    args = parser.parse_args()

    eval_tool = KittiEvalOdom()
    gt_file = args.gt_file
    est_file = args.est_file

    eval_tool.eval_single(
        args.gt_file,
        args.est_file,
        args.output_dir,
        args.seq_id,
        alignment=args.align,
    )
