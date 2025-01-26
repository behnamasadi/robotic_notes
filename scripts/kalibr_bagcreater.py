#!/usr/bin/env python

from __future__ import print_function
import os
import sys
import argparse
import math
import re

import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu

import cv2

import utility_functions

# case 1 create a rosbag from a folder as specified by the kalibr format
# https://github.com/ethz-asl/kalibr/wiki/bag-format
# structure
# dataset/cam0/TIMESTAMP.png
# dataset/camN/TIMESTAMP.png
# dataset/imu.csv

# case 2-4 create a rosbag from a video, a IMU file, and a video time file
# The video time file has the timestamp for every video frame per IMU clock.
# The saved bag will use the IMU clock to timestamp IMU and image messages.


def parse_args():
    parser = argparse.ArgumentParser(
        description='create a ROS bag containing image and imu topics '
        'from either image sequences or a video and inertial data.')
    # case 1 arguments used by the kalibr to create a rosbag from a folder
    parser.add_argument(
        '--folder',
        metavar='folder',
        nargs='?',
        help='Data folder whose content is structured as '
        'specified at\nhttps://github.com/ethz-asl/kalibr/wiki/bag-format')
    parser.add_argument('--output_bag',
                        default="output.bag",
                        help='ROS bag file %(default)s')

    # case 2 arguments to create a rosbag from a video, a IMU file,
    # and a video time file
    parser.add_argument('--video',
                        metavar='video_file',
                        nargs='?',
                        help='Video filename')
    parser.add_argument(
        '--imu',
        metavar='imu_file',
        nargs='+',
        help='Imu filename. If only one imu file is provided, '
        'then except for the optional header, '
        'each line format\n'
        'time[sec or nanosec], gx[rad/s], gy[rad/s], gz[rad/s],'
        ' ax[m/s^2], ay[m/s^2], az[m/s^2]. '
        'If gyro file and accelerometer file is provided,'
        'Each row should be: time[sec or nanosec],x,y,z'
        ' then accelerometer data will be interpolated'
        ' for gyro timestamps.')
    parser.add_argument('--first_frame_imu_time',
                        type=float,
                        default=0.0,
                        help='The time of the first video frame based on the'
                             ' Imu clock. It is used together with frame rate'
                             ' to estimate frame timestamps if the video time'
                             ' file is not provided. (default: %(default)s)',
                        required=False)
    parser.add_argument('--video_file_time_offset',
                        type=float,
                        default=0.0,
                        help='When the time file for the video is provided, '
                             'the video_file_time_offset may be added to '
                             'these timestamps.(default: %(default)s)',
                        required=False)
    parser.add_argument('--video_from_to',
                        type=float,
                        nargs=2,
                        help='Use the video frames starting from up to this'
                        ' time [s] relative to the video beginning.')
    parser.add_argument('--video_time_file',
                        default='',
                        nargs='?',
                        help='The csv file containing timestamps of every '
                        'video frames in IMU clock(default: %(default)s).'
                        ' Except for the header, each row has the '
                        'timestamp in nanosec as its first component',
                        required=False)
    parser.add_argument(
        '--max_video_frame_height',
        type=int,
        default=100000,
        help='For a video frame, if min(rows, cols) > %(default)s, '
        'it will be downsampled by 2. If the resultant bag is '
        'used for photogrammetry, the original focal_length and '
        'principal_point should be half-sized, but the '
        'distortion parameters should not be changed. (default: %(default)s)',
        required=False)
    parser.add_argument(
        "--shift_secs",
        type=float,
        default=0,
        help="shift all the measurement timestamps by this amount "
        "to avoid ros time starting from 0."
        "Only for case 4, see help.")

    if len(sys.argv) < 2:
        msg = 'Example usage 1: {} --folder kalibr/format/dataset ' \
              '--output_bag awsome.bag\n'.format(sys.argv[0])

        msg += 'Example usage 1.1: {} --folder tango/android/export/ ' \
               '--imu gyro_accel.csv ' \
               '--output_bag awsome.bag\n'.format(sys.argv[0])

        msg += "dataset or export has at least one subfolder cam0 which " \
               "contains nanosecond named images"

        msg += 'Example usage 2: {} --video marslogger/ios/dataset/' \
               'IMG_2805.MOV --imu marslogger/ios/dataset/gyro_accel.csv ' \
               '--video_time_file marslogger/ios/dataset/movie_metadata.csv ' \
               '--output_bag marslogger/ios/dataset/IMG_2805.bag\n'. \
            format(sys.argv[0])

        msg += 'Example usage 3: {} --video marslogger/android/dataset/' \
               'IMG_2805.MOV --imu marslogger/android/dataset/gyro_accel.csv' \
               ' --video_time_file ' \
               'marslogger/android/dataset/frame_timestamps.txt ' \
               '--output_bag marslogger/android/dataset/IMG_2805.bag\n '. \
            format(sys.argv[0])

        msg += 'Example usage 4: {} --video advio-01/iphone/frames.MOV --imu' \
               ' advio-01/iphone/gyro.csv advio-01/iphone/accelerometer.csv' \
               ' --video_time_file advio-01/iphone/frames.csv ' \
               '--shift_secs=100 --output_bag advio-01/iphone/iphone.bag\n '.\
            format(sys.argv[0])

        msg += ('For case 2 - 4, the first column of video_time_file should be'
                ' timestamp in sec or nanosec. Also the number of entries in '
                'video_time_file excluding its header lines has to be the '
                'same as the number of frames in the video.\nOtherwise, '
                'exception will be thrown. If the video and IMU data are'
                ' captured by a smartphone, then the conventional camera '
                'frame (C) and IMU frame (S) on a smartphone approximately '
                'satisfy R_SC = [0, -1, 0; -1, 0, 0; 0, 0, -1] with '
                'p_S = R_SC * p_C')

        print(msg)
        parser.print_help()
        sys.exit(1)

    parsed = parser.parse_args()
    return parsed


def get_image_files_from_dir(input_dir):
    """Generates a list of files from the directory"""
    image_files = list()
    timestamps = list()
    if os.path.exists(input_dir):
        for path, _, files in os.walk(input_dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg', '.pnm']:
                    image_files.append(os.path.join(path, f))
                    timestamps.append(os.path.splitext(f)[0])
    # sort by timestamp
    sort_list = sorted(zip(timestamps, image_files))
    image_files = [time_file[1] for time_file in sort_list]
    return image_files


def get_cam_folders_from_dir(input_dir):
    """Generates a list of all folders that start with cam e.g. cam0"""
    cam_folders = list()
    if os.path.exists(input_dir):
        for rootdir, folders, _ in os.walk(input_dir):
            for folder in folders:
                if folder[0:3] == "cam":
                    cam_folders.append(os.path.join(rootdir, folder))
    return cam_folders


def get_imu_csv_files(input_dir):
    """Generates a list of all csv files that start with imu"""
    imu_files = list()
    if os.path.exists(input_dir):
        for path, _, files in os.walk(input_dir):
            for filename in files:
                if filename[0:3] == 'imu' and os.path.splitext(
                        filename)[1] == ".csv":
                    imu_files.append(os.path.join(path, filename))

    return imu_files


def load_image_to_ros_msg(filename, timestamp=None):
    image_np = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    if timestamp is None:
        timestamp_nsecs = os.path.splitext(os.path.basename(filename))[0]
        timestamp = rospy.Time(secs=int(timestamp_nsecs[0:-9]),
                               nsecs=int(timestamp_nsecs[-9:]))

    rosimage = Image()
    rosimage.header.stamp = timestamp
    rosimage.height = image_np.shape[0]
    rosimage.width = image_np.shape[1]
    # only with mono8! (step = width * byteperpixel * numChannels)
    rosimage.step = rosimage.width
    rosimage.encoding = "mono8"
    rosimage.data = image_np.tostring()

    return rosimage, timestamp


def create_rosbag_for_images_in_dir(data_dir, output_bag, topic = "/cam0/image_raw"):
    """
    Find all images recursively in data_dir, and put them in a rosbag under topic.
    The timestamp for each image is determined by its index.
    It is mainly used to create a rosbag for calibrating cameras with kalibr.
    :param data_dir:
    :param output_bag: Its existing content will be overwritten.
    :param topic:
    :return:
    """
    image_files=get_image_files_from_dir(data_dir)
    print('Found #images {} under {}'.format(len(image_files), data_dir))
    bag = rosbag.Bag(output_bag, 'w')
    for index, image_filename in enumerate(image_files):
        image_msg, timestamp = load_image_to_ros_msg(image_filename, rospy.Time(index + 1, 0))
        bag.write(topic, image_msg, timestamp)
    print("Saved #images {} of to bag".format(len(image_files)))
    bag.close()


def create_imu_message_time_string(timestamp_str, omega, alpha):
    secs, nsecs = utility_functions.parse_time(timestamp_str, 'ns')
    timestamp = rospy.Time(secs, nsecs)
    return create_imu_message(timestamp, omega, alpha), timestamp


def create_imu_message(timestamp, omega, alpha):
    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.angular_velocity.x = float(omega[0])
    rosimu.angular_velocity.y = float(omega[1])
    rosimu.angular_velocity.z = float(omega[2])
    rosimu.linear_acceleration.x = float(alpha[0])
    rosimu.linear_acceleration.y = float(alpha[1])
    rosimu.linear_acceleration.z = float(alpha[2])
    return rosimu


def write_video_to_rosbag(bag,
                          video_filename,
                          video_from_to,
                          first_frame_imu_time=0.0,
                          frame_timestamps=None,
                          frame_remote_timestamps=None,
                          max_video_frame_height=100000,
                          shift_in_time=0.0,
                          topic="/cam0/image_raw",
                          ratio=1.0):
    """
    :param bag: opened bag stream writing to
    :param video_filename:
    :param video_from_to: start and finish seconds within the video. Only frames within this range will be bagged.
    :param first_frame_imu_time: The rough timestamp of the first frame per the IMU clock.
    This value is used to estimate the video time range per the IMU clock.
    Also it is used for frame local time if frame_timestamps is empty.
    :param frame_timestamps: Frame timestamps by the host clock.
    :param frame_remote_timestamps: Frame timestamps by the remote device clock.
    :param max_video_frame_height:
    :param shift_in_time: The time shift to apply to the resulting local (host) timestamps.
    :param ratio: ratio of output frames / input frames
    :return: rough video time range in imu clock.
    first_frame_imu_time + frame_time_in_video(0 based) ~= frame_time_in_imu.
    """
    cap = cv2.VideoCapture(video_filename)
    rate = cap.get(cv2.CAP_PROP_FPS)
    print("video frame rate {}".format(rate))

    start_id = 0
    finish_id = 1000000
    image_time_range_in_bag = list()
    framesinvideo = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print('#Frames {} in the video {}'.format(framesinvideo, video_filename))
    if frame_timestamps is None:
        frame_timestamps = list()
    if frame_timestamps:
        if len(frame_timestamps) != framesinvideo:
            raise Exception((
                "Number of frames in the video {} disagrees with the length of"
                " the provided timestamps {}").format(framesinvideo,
                                                      len(frame_timestamps)))
    if finish_id == -1:
        finish_id = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) - 1)
    else:
        finish_id = int(min(finish_id, framesinvideo - 1))
    if video_from_to and rate > 1.0:
        start_id = int(max(start_id, video_from_to[0] * rate))
        finish_id = int(min(finish_id, video_from_to[1] * rate))
    image_time_range_in_bag.append(
        max(float(start_id) / rate - 0.05, 0.0) + first_frame_imu_time +
        shift_in_time)
    image_time_range_in_bag.append(
        float(finish_id) / rate + 0.05 + first_frame_imu_time + shift_in_time)
    print('video frame index start {} finish {}'.format(start_id, finish_id))
    cap.set(cv2.CAP_PROP_POS_FRAMES,
            start_id)  # start from start_id, 0 based index
    current_id = start_id
    framecount = 0
    last_frame_remote_time = 0
    while cap.isOpened():
        if current_id > finish_id:
            print('Exceeding finish_id %d' % finish_id +
                  ', break the video stream')
            break

        video_frame_id = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        if video_frame_id != current_id:
            message = "Expected frame id {} and actual id in video {} " \
                      "differ.\n".format(current_id, video_frame_id)
            message += "Likely reached end of video file. Note finish_id is " \
                       "{}".format(finish_id)
            print(message)
            break

        time_frame = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        time_frame_offset = time_frame + first_frame_imu_time

        _, frame = cap.read()
        if frame is None:
            print('Empty frame, break the video stream')
            break

        image_np = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = image_np.shape[:2]
        if min(w, h) > max_video_frame_height:
            image_np = cv2.pyrDown(image_np, dstsize=(w // 2, h // 2))
        if w < h:
            image_np = cv2.rotate(image_np, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow('frame', image_np)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if frame_timestamps:  # external
            local_time = frame_timestamps[video_frame_id]
        else:
            decimal, integer = math.modf(time_frame_offset)
            local_time = rospy.Time(secs=int(integer), nsecs=int(decimal * 1e9))
        local_time += rospy.Duration.from_sec(shift_in_time)

        remote_time = local_time
        is_dud = False
        if frame_remote_timestamps:
            remote_time = frame_remote_timestamps[video_frame_id]
            if last_frame_remote_time != 0:
                is_dud = remote_time == last_frame_remote_time
            last_frame_remote_time = remote_time
        ratiostatus = (current_id - start_id) * ratio >= framecount
        if ratiostatus and not is_dud:
            rosimage = Image()
            rosimage.header.stamp = remote_time
            rosimage.height = image_np.shape[0]
            rosimage.width = image_np.shape[1]
            rosimage.step = rosimage.width
            rosimage.encoding = "mono8"
            rosimage.data = image_np.tostring()
            bag.write(topic, rosimage, local_time)
            framecount += 1
        else:
            print('Skip frame of id {}, is dud? {} ratio status? {} start_id {} framecount {}.'.format(
                current_id, is_dud, ratiostatus, start_id, framecount))

        current_id += 1

    cap.release()
    cv2.destroyAllWindows()
    print('Saved {} out of {} video frames as image messages to the rosbag'.
          format(framecount, framesinvideo))
    return image_time_range_in_bag


def loadtimestamps(time_file):
    """Except for the header, each row has the timestamp in nanosec
    as its first component"""
    frame_rostimes = list()

    # https://stackoverflow.com/questions/41847146/multiple-delimiters-in-single-csv-file
    with open(time_file, 'r') as stream:
        for line in stream:
            line = line.strip()
            row = re.split(' |,|[|]', line)
            if utility_functions.is_header_line(row[0]):
                continue
            secs, nsecs = utility_functions.parse_time(row[0], 'ns')
            frame_rostimes.append(rospy.Time(secs, nsecs))
    return frame_rostimes


def load_local_and_remote_times(time_file):
    """

    :param time_file: Except for the header lines, each line has the first column as the remote time in ns,
    and the last column as the local time in secs.
    :return: list of tuples (local time, remote time)
    """
    frame_rostimes = list()

    with open(time_file, 'r') as stream:
        for line in stream:
            row = re.split(' |,|[|]', line)
            if utility_functions.is_header_line(row[0]):
                continue
            secs, nsecs = utility_functions.parse_time(row[0], 'ns')
            remote_time = rospy.Time(secs, nsecs)
            local_time = rospy.Time.from_sec(float(row[-1]))
            frame_rostimes.append((local_time, remote_time))
    return frame_rostimes


def write_imufile_to_rosbag(bag, imufile, timerange=None, buffertime=5, topic="/imu0"):
    """

    :param bag: output bag stream.
    :param imufile: each line: time(ns), gx, gy, gz, ax, ay, az
    :param timerange: only write imu data within this range to the bag, in seconds.
    :param buffertime: time to pad the timerange, in seconds.
    :param topic: imu topic
    :return:
    """
    utility_functions.check_file_exists(imufile)

    with open(imufile, 'r') as stream:
        rowcount = 0
        imucount = 0
        for line in stream:
            row = re.split(' |,|[|]', line)  # note a row is a list of strings.
            if utility_functions.is_header_line(row[0]):
                continue
            imumsg, timestamp = create_imu_message_time_string(
                row[0], row[1:4], row[4:7])
            timestampinsec = timestamp.to_sec()
            rowcount += 1
            if timerange and \
                    (timestampinsec < timerange[0] - buffertime or
                     timestampinsec > timerange[1] + buffertime):
                continue
            imucount += 1
            bag.write(topic, imumsg, timestamp)
        print('Saved {} out of {} inertial messages to the rosbag'.format(
            imucount, rowcount))


def write_imufile_remotetime_to_rosbag(bag, imufile, timerange=None, buffertime=5, topic="/imu0"):
    """

    :param bag: output bag stream.
    :param imufile: each line: time(sec), gx, gy, gz, ax, ay, az, mx, my, mz, device-time(sec), and others
    :param timerange: only write imu data within this local time range to the bag, in seconds.
    :param buffertime: time to pad the timerange, in seconds.
    :param topic: imu topic
    :return:
    """
    utility_functions.check_file_exists(imufile)

    with open(imufile, 'r') as stream:
        rowcount = 0
        imucount = 0
        lastRemoteTime = rospy.Time(0, 0)
        for line in stream:
            row = re.split(' |,|[|]', line)  # note a row is a list of strings.
            if utility_functions.is_header_line(row[0]):
                continue
            local_timestamp = rospy.Time.from_sec(float(row[0]))
            remote_timestamp = rospy.Time.from_sec(float(row[10]))
            assert remote_timestamp > lastRemoteTime, \
                "remote time is not increasing in {}. Is the index of the remote time 10?".format(imufile)
            lastRemoteTime = remote_timestamp

            imumsg = create_imu_message(remote_timestamp, row[1:4], row[4:7])
            timestampinsec = local_timestamp.to_sec()
            rowcount += 1
            if timerange and \
                    (timestampinsec < timerange[0] - buffertime or
                     timestampinsec > timerange[1] + buffertime):
                continue
            imucount += 1
            bag.write(topic, imumsg, local_timestamp)

        print('Saved {} out of {} inertial messages to the rosbag'.format(
            imucount, rowcount))


def write_gyro_accel_to_rosbag(bag, imufiles, timerange=None, buffertime=5, topic="/imu0", shift_secs=0.0):
    gyro_file = imufiles[0]
    accel_file = imufiles[1]
    for filename in imufiles:
        utility_functions.check_file_exists(filename)
    time_gyro_array = utility_functions.load_advio_imu_data(gyro_file)
    time_accel_array = utility_functions.load_advio_imu_data(accel_file)
    time_imu_array = utility_functions.interpolate_imu_data(
        time_gyro_array, time_accel_array)
    bag_imu_count = 0
    for row in time_imu_array:
        timestamp = rospy.Time.from_sec(row[0]) + rospy.Duration.from_sec(shift_secs)
        imumsg = create_imu_message(timestamp, row[1:4], row[4:7])

        timestampinsec = timestamp.to_sec()
        # check below conditions when video and imu use different clocks
        # and their lengths differ much
        if timerange and \
                (timestampinsec < timerange[0] - buffertime or
                 timestampinsec > timerange[1] + buffertime):
            continue
        bag_imu_count += 1
        bag.write(topic, imumsg, timestamp)
    print('Saved {} out of {} inertial messages to the rosbag'.format(
        bag_imu_count, time_imu_array.shape[0]))


def main():
    parsed = parse_args()

    bag = rosbag.Bag(parsed.output_bag, 'w')
    videotimerange = None  # time range of video frames in IMU clock
    if parsed.video is not None:
        utility_functions.check_file_exists(parsed.video)
        print('Given video time offset {}'.format(parsed.first_frame_imu_time))
        if parsed.video_from_to:
            print('Frame time range within the video: {}'.format(parsed.video_from_to))
        first_frame_imu_time = parsed.first_frame_imu_time
        frame_timestamps = list()
        if parsed.video_time_file:
            frame_timestamps = loadtimestamps(parsed.video_time_file)
            aligned_timestamps = [time + rospy.Duration.from_sec(parsed.video_file_time_offset)
                                  for time in frame_timestamps]
            frame_timestamps = aligned_timestamps
            print('Loaded {} timestamps for frames'.format(
                len(frame_timestamps)))
            first_frame_imu_time = frame_timestamps[0].to_sec()
        videotimerange = write_video_to_rosbag(
            bag,
            parsed.video,
            parsed.video_from_to,
            first_frame_imu_time,
            frame_timestamps,
            frame_remote_timestamps=None,
            max_video_frame_height=parsed.max_video_frame_height,
            shift_in_time=parsed.shift_secs,
            topic="/cam0/image_raw")

    elif parsed.folder is not None:
        # write images
        camfolders = get_cam_folders_from_dir(parsed.folder)
        for camdir in camfolders:
            camid = os.path.basename(camdir)
            image_files = get_image_files_from_dir(camdir)
            for image_filename in image_files:
                image_msg, timestamp = load_image_to_ros_msg(image_filename)
                bag.write("/{0}/image_raw".format(camid), image_msg,
                          timestamp)
            print("Saved #images {} of {} to bag".format(
                len(image_files), camid))
    else:
        raise Exception('Invalid/Empty video file and image folder')

    # write imu data
    if (not parsed.imu) and parsed.folder is None:
        print("Neither a folder nor any imu file is provided. "
              "Rosbag will have only visual data")
    elif not parsed.imu:
        imufiles = get_imu_csv_files(parsed.folder)
        for imufile in imufiles:
            topic = os.path.splitext(os.path.basename(imufile))[0]
            with open(imufile, 'r') as stream:
                for line in stream:
                    row = re.split(' |,|[|]', line)
                    if utility_functions.is_header_line(row[0]):
                        continue
                    imumsg, timestamp = create_imu_message_time_string(
                        row[0], row[1:4], row[4:7])
                    bag.write("/{0}".format(topic), imumsg, timestamp)
    elif len(parsed.imu) == 1:
        write_imufile_to_rosbag(bag, parsed.imu[0], videotimerange, 5, "/imu0")
    else:
        write_gyro_accel_to_rosbag(bag, parsed.imu, videotimerange, 5, "/imu0", parsed.shift_secs)

    bag.close()
    print('Saved to bag file {}'.format(parsed.output_bag))


if __name__ == "__main__":
    main()
