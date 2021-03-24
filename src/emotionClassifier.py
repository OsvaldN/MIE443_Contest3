#!/usr/bin/env python
from __future__ import print_function
import torch
import cv2
import torchvision
import numpy as np
import rospy
import roslib # Needed to import ModelStates
import argparse
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from emotionTrainingSample import EmotionClassificationNet
from mie443_contest3.msg import EmotionMsg
from mie443_contest3.msg import EmotionFaceMsg
import matplotlib.pyplot as plt

class EmotionDetector(object):

    def __init__(self, args):
        #
        # Python 2.7 syntax.
        super(EmotionDetector, self).__init__()
        #
        # Load your emotion detector.
        self.model = EmotionClassificationNet()
        self.model.load_state_dict(torch.load(args.model_file))
        self.model.eval()
        #
        # Visualize.
        self.vis = args.vis
        print('Setting up subscribers.')
        self.emotion_sub = rospy.Subscriber('/emotion_img', EmotionFaceMsg, self.emotionsub)
        self.emotion_pub = rospy.Publisher('/detected_emotion', Int32, self.emotionsub, queue_size=1)
        self.emotion_file = open('detectedVictim.txt', 'w')

    def showImBatch(self, imgs):
        img_grid = torchvision.utils.make_grid(imgs)
        self.matplotlib_imshow(img_grid)

    def matplotlib_imshow(self, img):
        img2 = img + 0.5     # uncenter
        npimg = img.numpy().transpose([1,2,0])
        cv2.imshow('Input', npimg)
        cv2.waitKey(0)

    def emotionsub(self, msg):
        with torch.no_grad():
            imgs = msg.data
            w = msg.width
            h = msg.height
            b = msg.batch
            imgs = torch.from_numpy(np.array(imgs)).view(b, 1, h, w).float()
            if self.vis:
                print('Showing images.')
                self.showImBatch(imgs)
            emotions = self.model(imgs, True)
            #
            # Emotion voting -- take the most often voted for emotion, ties are broken arbitrarily.
            uniqueEmotions, counts = emotions.unique(sorted=True, return_counts=True)
            print('uniqueEmotions:', uniqueEmotions)
            print('EmotionCounts:', counts)
            cnt_max, max_idx = counts.max(0)
            print(uniqueEmotions[max_idx])
            intmsg = Int32()
            intmsg.data = uniqueEmotions[max_idx].item()
            self.emotion_pub.publish(intmsg)
            self.emotion_file.write(str(uniqueEmotions[max_idx].item()))

    def logEmotionHistory(self):
        self.emotion_file.close()
#
# Parse the input arguments.
def getInputArgs():
    parser = argparse.ArgumentParser('MIE443_contest3 victim emotion detector.')
    parser.add_argument('--gpu', dest='gpu', default=torch.cuda.is_available(), type=bool, help='Use gpu for training')
    parser.add_argument('--model', dest='model_file', default='mdl_best.pth', type=str, help='NN model to use for emotion detection.')
    parser.add_argument('--vis', dest='vis', default=False, action='store_true', help='Visualize the received images.')
    args = parser.parse_args()
    return args
#
#
if __name__ == "__main__":
    rospy.init_node('emotionDetector')
    args = getInputArgs()
    victim_locations = EmotionDetector(args)
    rospy.spin()
