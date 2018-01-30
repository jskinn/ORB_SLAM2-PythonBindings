#!/usr/bin/env python3
import sys
import os.path
import orbslam2
import time
import cv2


def main(vocab_path, settings_path, sequence_path):

    left_filenames, right_filenames, timestamps = load_images(sequence_path)
    num_images = len(timestamps)

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

        t1 = time.time()
        slam.process_image_stereo(left_image[:, :, ::-1], right_image[:, :, ::-1], tframe)
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


def load_images(path_to_sequence):
    timestamps = []
    with open(os.path.join(path_to_sequence, 'times.txt')) as times_file:
        for line in times_file:
            if len(line) > 0:
                timestamps.append(float(line))

    return [
        os.path.join(path_to_sequence, 'image_0', "{0:06}.png".format(idx))
        for idx in range(len(timestamps))
    ], [
        os.path.join(path_to_sequence, 'image_1', "{0:06}.png".format(idx))
        for idx in range(len(timestamps))
    ], timestamps


def save_trajectory(trajectory, filename):
    with open(filename, 'w') as traj_file:
        traj_file.writelines('{time} {r00} {r01} {r02} {t0} {r10} {r11} {r12} {t1} {r20} {r21} {r22} {t2}\n'.format(
            time=repr(stamp),
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
        ) for stamp, r00, r01, r02, t0, r10, r11, r12, t1, r20, r21, r22, t2 in trajectory)


if __name__ == '__main__':
    if len(sys.argv) != 4:
        print('Usage: ./orbslam_stereo_kitti path_to_vocabulary path_to_settings path_to_sequence')
    main(sys.argv[1], sys.argv[2], sys.argv[3])
