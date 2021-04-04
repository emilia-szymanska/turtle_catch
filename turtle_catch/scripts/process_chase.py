#!/usr/bin/env python3
import rosbag
import sys
import numpy as np
CHASER_POSE_TOPIC = '/hunter/pose'
RUNNER_POSE_TOPIC = '/turtle1/pose'


def covered_dist(pose_list):
    first = True
    dist = 0
    for pose in pose_list:
        if first:
            first = False
            previous = np.array((pose.x, pose.y))
        else:
            current = np.array((pose.x, pose.y))
            dist += np.linalg.norm(current-previous)
            previous = current
    return dist


def printing_output(runner_dist, runner_vel, chaser_dist, chaser_vel, chaser_t):
    print('Runner')
    print(f'  Covered distance: {runner_dist:.2f} m')
    print(f'  Average velocity: {runner_vel:.2f} m/s')

    print('Chaser')
    print(f'  Covered distance: {chaser_dist:.2f} m')
    print(f'  Average velocity: {chaser_vel:.2f} m/s')

    print(f'Chase duration: {chaser_t:.2f} s')


def process_bag(outbag_filename, inbag_filename):
    msg_counter = 0
    chaser_poses = []
    runner_poses = []
    last_t_chaser = 0
    last_t_runner = 0
    first_t_chaser = 0
    first_t_runner = 0
    first_chaser = True
    first_runner = True
    
    with rosbag.Bag(outbag_filename, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(inbag_filename, 'r').read_messages():
            if topic == CHASER_POSE_TOPIC:
                if first_chaser:
                    first_t_chaser = t
                    first_chaser = False
                outbag.write('/chaser/pose', msg, t)
                chaser_poses.append(msg)
                last_t_chaser = t
                msg_counter += 1
            elif topic == RUNNER_POSE_TOPIC:
                if first_runner:
                    first_t_runner = t
                    first_runner = False
                outbag.write('/runner/pose', msg, t)
                runner_poses.append(msg)
                last_t_runner = t
                msg_counter += 1

    runner_t = (last_t_runner - first_t_runner).to_sec()
    chaser_t = (last_t_chaser - first_t_chaser).to_sec()
    

    runner_dist = covered_dist(runner_poses)
    chaser_dist = covered_dist(chaser_poses)
    
    chaser_vel = chaser_dist / chaser_t
    runner_vel = runner_dist / runner_t
    
    return msg_counter, runner_dist, runner_vel, chaser_dist, chaser_vel, chaser_t



if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f'Usage: {sys.argv[0]} input.bag')
        sys.exit()

    inbag_filename = sys.argv[1]
    outbag_filename = 'processed_chase.bag'

    print(f'Processing input bagfile: {inbag_filename}')

    msg_counter, runner_dist, runner_vel, chaser_dist, chaser_vel, chaser_t = process_bag(outbag_filename, inbag_filename)
    printing_output(runner_dist, runner_vel, chaser_dist, chaser_vel, chaser_t)
    print(f'Wrote {msg_counter} messages to {outbag_filename}')

