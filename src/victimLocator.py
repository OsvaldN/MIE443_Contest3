#!/usr/bin/env python
from __future__ import print_function
import os
import torch
import rospy
import roslib # Needed to import ModelStates
import random
import argparse
import numpy as np
import pandas as pd
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from mie443_contest3.msg import EmotionFaceMsg
#
# Parse the input arguments.
def getInputArgs():
    parser = argparse.ArgumentParser('MIE443_contest3 victim detector.')
    parser.add_argument('--victim_file', dest='victim_file', default='victims.csv', type=str, help='Locations of the victims in the environment')
    parser.add_argument('--emotion_file', dest='emotion_file', default='train_split.pth', type=str, help='Emotion detection file')
    parser.add_argument('--n_face_samples', dest='n_face_samples', default=10, type=int, help='Emotion detection file')
    args = parser.parse_args()
    return args

class VictimLocations(object):

    def __init__(self, args):
        #
        # Setup victims.
        victims = pd.read_csv(os.path.abspath(args.victim_file))
        victims.columns = victims.columns.str.strip()
        self.pub_victim_count = 0

        assert set(['pos_x', 'pos_y', 'radius']).issubset(set(victims.columns.astype(str).tolist()))
        self.results_file = open('detectedVictim.txt', 'w')
        self.victim_pose = victims.as_matrix(['pos_x', 'pos_y'])
        self.victim_det_rad = victims.as_matrix(['radius']).reshape(-1)
        self.publish_victims(self.victim_pose, self.victim_det_rad)
        #
        # Setup emotions.
        self.n_face_samples = args.n_face_samples
        self.emotion_lists, self.possible_emotions = self.loadEmotion(args.emotion_file)
        random.shuffle(self.possible_emotions)
        rospy.on_shutdown(self.logVictimHistory)
        #
        # Subscribe to the location of the robot.
        self.gazebo_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.stateSub)
        self.emotion_pub = rospy.Publisher('/emotion_img', EmotionFaceMsg, queue_size=1)
        while self.emotion_pub.get_num_connections() == 0:
            print('Waiting for connection for emotion pub.')
            rospy.sleep(0.5)

    def publish_victims(self, pose, rad):
        markerPub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        while markerPub.get_num_connections() == 0:
            print('Waiting for connection for marker pub.')
            rospy.sleep(0.5)
        msg = ModelState()
        msg.reference_frame = 'world'
        for v_idx in range(pose.shape[0]):
            msg.model_name = 'box_contest3_clone_' + str(v_idx)
            msg.pose.position.x = pose[v_idx][0]
            msg.pose.position.y = pose[v_idx][1]
            msg.pose.position.z = -0.999
            msg.pose.orientation.x = -0.5
            msg.pose.orientation.y = -0.5
            msg.pose.orientation.z = -0.5
            msg.pose.orientation.w = 0.5
            markerPub.publish(msg)

    def loadEmotion(self, emotion_file_path):
        imgs, labels = torch.load(os.path.abspath(emotion_file_path))
        unique_emotions = labels.unique()
        emotion_imgs = {}
        for l in unique_emotions:
            emotion_mask = labels == l
            emotion_imgs[l.item()] = imgs[emotion_mask].numpy()
        return emotion_imgs, unique_emotions.numpy().tolist()

    def robotLocation(self, msg):
        assert isinstance(msg, Pose)
        return np.array([msg.position.x, msg.position.y])

    def pubEmotionSet(self, gt_emotion_idx):
        print('Publishing emotion images')
        emo_imgs = self.emotion_lists[gt_emotion_idx]
        selected_emo_imgs = emo_imgs[np.random.choice(emo_imgs.shape[0], self.n_face_samples, replace=False), :]
        print(selected_emo_imgs.shape)
        emo_msg = EmotionFaceMsg()
        emo_msg.width = 48
        emo_msg.height = 48
        emo_msg.batch = self.n_face_samples
        emo_msg.data = selected_emo_imgs.reshape(-1,1,1)
        while self.emotion_pub.get_num_connections() == 0:
            print('Waiting for connection for emotion pub.')
            rospy.sleep(0.05)
        self.emotion_pub.publish(emo_msg)
        print('Published emotion images')

    def stateSub(self, msg):
        #
        # This publish command is unreliable, but needed to ensure the markers are in the ground and in place.
        if self.pub_victim_count < 200:
            self.publish_victims(self.victim_pose, self.victim_det_rad)
            self.pub_victim_count += 1
        for idx, name in enumerate(msg.name):
            if name == 'mobile_base':
                break
        else:
            return
        #
        # Check if the current state of the robot intersects with any victim.
        rpose = self.robotLocation(msg.pose[idx])
        vic_dist = np.sqrt(np.sum((rpose - self.victim_pose) ** 2, -1)).reshape(-1)
        vic_detected = vic_dist < self.victim_det_rad
        detected_poses = self.victim_pose[vic_detected]
        #
        # Clean up what has been detected.
        self.victim_det_rad = self.victim_det_rad[~vic_detected]
        self.victim_pose = self.victim_pose[~vic_detected]
        for pose in detected_poses:
            gt_emotion_idx = self.possible_emotions.pop(0)
            print('Sending emotion:', gt_emotion_idx)
            detected_victim = [pose, gt_emotion_idx]
            self.pubEmotionSet(gt_emotion_idx)
            self.results_file.write(str(detected_victim))

    def logVictimHistory(self):
        self.results_file.close()

if __name__ == "__main__":
    rospy.init_node('victimLocator')
    args = getInputArgs()
    victim_locations = VictimLocations(args)
    rospy.spin()
