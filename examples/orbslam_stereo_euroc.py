#!/usr/bin/env python3
import sys
import os.path
import numpy as np
import orbslam2
import time
import cv2


def main(vocab_path, settings_path, path_to_left_folder, path_to_right_folder, path_to_times_file):

    left_filenames, right_filenames, timestamps = load_images(path_to_left_folder, path_to_right_folder,
                                                              path_to_times_file)

    # read stereo rectification parameters from settings
    m1l, m2l, m1r, m2r = load_stereo_rectification(settings_path)

    num_images = len(left_filenames)

    slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.STEREO)
    slam.set_use_viewer(True)
    slam.initialize()

    times_track = [0 for _ in range(num_images)]
    print('-----')
    print('Start processing sequence ...')
    print('Images in the sequence: {0}'.format(num_images))

    for idx in range(num_images):
        left_image = cv2.imread(left_filenames[idx], cv2.IMREAD_UNCHANGED)
        right_image = cv2.imread(right_filenames[idx], cv2.IMREAD_UNCHANGED)
        tframe = timestamps[idx]

        if left_image is None:
            print("failed to load image at {0}".format(left_filenames[idx]))
            return 1
        if right_image is None:
            print("failed to load image at {0}".format(right_filenames[idx]))
            return 1

        left_image = cv2.remap(left_image, m1l, m2l, cv2.INTER_LINEAR)
        right_image = cv2.remap(right_image, m1r, m2r, cv2.INTER_LINEAR)

        t1 = time.time()
        slam.process_image_stereo(left_image, right_image, tframe)
        t2 = time.time()

        ttrack = t2 - t1
        times_track[idx] = ttrack

        t = 0
        if idx < num_images - 1:
            t = timestamps[idx + 1] - tframe
        elif idx > 0:
            t = tframe - timestamps[idx - 1]

        if ttrack < t:
            time.sleep(t - ttrack)

    save_trajectory(slam.get_trajectory_points(), 'trajectory.txt')

    slam.shutdown()

    times_track = sorted(times_track)
    total_time = sum(times_track)
    print('-----')
    print('median tracking time: {0}'.format(times_track[num_images // 2]))
    print('mean tracking time: {0}'.format(total_time / num_images))

    return 0


def load_images(path_to_left, path_to_right, path_to_times_file):
    left_files = []
    right_files = []
    timestamps = []
    with open(path_to_times_file) as times_file:
        for line in times_file:
            line = line.rstrip()
            timestamps.append(float(line) / 1e9)
            left_files.append(os.path.join(path_to_left, "{0}.png".format(line)))
            right_files.append(os.path.join(path_to_right, "{0}.png".format(line)))
    return left_files, right_files, timestamps


def load_stereo_rectification(settings_path):
    settings = cv2.FileStorage(settings_path, cv2.FileStorage_READ)
    K_l = settings.getNode('LEFT.K').mat()
    K_r = settings.getNode('RIGHT.K').mat()
    P_l = settings.getNode('LEFT.P').mat()
    P_r = settings.getNode('RIGHT.P').mat()
    R_l = settings.getNode('LEFT.R').mat()
    R_r = settings.getNode('RIGHT.R').mat()
    D_l = settings.getNode('LEFT.D').mat()
    D_r = settings.getNode('RIGHT.D').mat()
    rows_l = int(settings.getNode('LEFT.height').real())
    cols_l = int(settings.getNode('LEFT.width').real())
    rows_r = int(settings.getNode('RIGHT.height').real())
    cols_r = int(settings.getNode('RIGHT.width').real())

    m1l, m2l = cv2.initUndistortRectifyMap(K_l, D_l, R_l, P_l[0:3, 0:3], (cols_l, rows_l), cv2.CV_32F)
    m1r, m2r = cv2.initUndistortRectifyMap(K_r, D_r, R_r, P_r[0:3, 0:3], (cols_r, rows_r), cv2.CV_32F)
    return m1l, m2l, m1r, m2r


def load_cv_mat(yaml_mat):
    return np.array(yaml_mat)


def save_trajectory(trajectory, filename):
    with open(filename, 'w') as traj_file:
        traj_file.writelines('{time} {r00} {r01} {r02} {t0} {r10} {r11} {r12} {t1} {r20} {r21} {r22} {t2}\n'.format(
            time=repr(t),
            r00=repr(r00),
            r01=repr(r01),
            r02=repr(r02),
            t0=repr(t0),
            r10=repr(r10),
            r11=repr(r11),
            r12=repr(r12),
            t1=repr(t1),
            r20=repr(r20),
            r21=repr(r21),
            r22=repr(r22),
            t2=repr(t2)
        ) for t, r00, r01, r02, t0, r10, r11, r12, t1, r20, r21, r22, t2 in trajectory)


if __name__ == '__main__':
    if len(sys.argv) != 6:
        print('Usage: ./orbslam_stereo_euroc path_to_vocabulary path_to_settings path_to_left_folder path_to_right_folder path_to_times_file')
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
