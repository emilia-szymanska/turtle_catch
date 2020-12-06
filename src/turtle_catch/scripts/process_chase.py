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


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f'Usage: {sys.argv[0]} input.bag')
        sys.exit()

    inbag_filename = sys.argv[1]
    outbag_filename = 'processed_chase.bag'

    print(f'Processing input bagfile: {inbag_filename}')
    msg_counter = 0

    #now = rospy.Time.now()
    # `now` and `begin` are rospy.Time objects. Their difference is a
    # rospy.Duration, which can be converted into a float seconds using
    # the `to_sec()` method.
    # See:
    # http://wiki.ros.org/rospy/Overview/Time
    #t = (now - initial_time).to_sec()
    chaser_poses = []
    runner_poses = []

    with rosbag.Bag(outbag_filename, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(inbag_filename, 'r').read_messages():
            if topic == CHASER_POSE_TOPIC:
                outbag.write('/chaser/pose', msg, t)
                chaser_poses.append(msg)
                msg_counter += 1
            elif topic == RUNNER_POSE_TOPIC:
                outbag.write('/runner/pose', msg, t)
                runner_poses.append(msg)
                msg_counter += 1

    runner_dist = round(covered_dist(runner_poses), 2)
    chaser_dist = round(covered_dist(chaser_poses), 2)
    
    print('Runner')
    print(f'  Covered distance: {runner_dist} m')

    print('Chaser')
    print(f'  Covered distance: {chaser_dist} m')

    print(f'Wrote {msg_counter} messages to {outbag_filename}')
