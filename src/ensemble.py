import torch
import torch.nn as nn
import numpy as np
import torch.nn.functional as F

class EmotionClassificationNet(nn.Module):

    def __init__(self, dropRate=0.25):
        super(EmotionClassificationNet, self).__init__()
        self.cnn = nn.Sequential(
            nn.Conv2d(1, 64, 3, padding=1),
            nn.LeakyReLU(),
            nn.BatchNorm2d(64),
            nn.MaxPool2d(2, 2),
            nn.Dropout(dropRate),

            nn.Conv2d(64, 128, 3, padding=1),
            nn.LeakyReLU(),
            nn.BatchNorm2d(128),
            nn.MaxPool2d(2, 2),
            nn.Dropout(dropRate),

            nn.Conv2d(128, 256, 3, padding=1),
            nn.LeakyReLU(),
            nn.BatchNorm2d(256),
            nn.MaxPool2d(2, 2),
            nn.Dropout(dropRate),

            nn.Conv2d(256, 512, 3, padding=1),
            nn.LeakyReLU(),
            nn.BatchNorm2d(512),
            nn.MaxPool2d(2, 2),
            nn.Dropout(dropRate),

            nn.Conv2d(512, 512, 3, padding=1),
            nn.LeakyReLU(),
            nn.BatchNorm2d(512),
            nn.Dropout(dropRate),
        )
        self.nn = nn.Sequential(
            nn.Linear(512*3*3, 512),
            nn.LeakyReLU(),
            nn.Dropout(dropRate),
            nn.Linear(512, 256),
            nn.LeakyReLU(),
            nn.Dropout(dropRate),
            nn.Linear(256, 7),
        )
        self.decode = nn.Sequential(
            nn.ConvTranspose2d(512, 256, 3, padding=1),
            nn.LeakyReLU(),
            nn.BatchNorm2d(256),
            nn.Upsample(scale_factor=2),

            nn.ConvTranspose2d(256, 128, 3, padding=1),
            nn.LeakyReLU(),
            nn.BatchNorm2d(128),
            nn.Upsample(scale_factor=2),
            
            nn.ConvTranspose2d(128, 64, 3, padding=1),
            nn.LeakyReLU(),
            nn.BatchNorm2d(64),
            nn.Upsample(scale_factor=2),

            nn.ConvTranspose2d(64, 1, 3, padding=1),
            nn.LeakyReLU(),
            nn.Upsample(scale_factor=2),
            nn.Sigmoid()
        )

    
    def forward(self, x, test_mode=False, reconstruct=False):
        batch_size = x.shape[0]
        x = self.cnn(x)
        
        if reconstruct:
            x = self.decode(x)
            return x
        
        out = self.nn(x.view(batch_size, -1))

        return out

class Ensemble(nn.Module):

    def __init__(self):
        super(Ensemble, self).__init__()
        self.cnn1 = EmotionClassificationNet(dropRate=0)
        self.cnn2 = EmotionClassificationNet(dropRate=0)
        self.cnn3 = EmotionClassificationNet(dropRate=0)
        self.cnn4 = EmotionClassificationNet(dropRate=0)
        self.cnn5 = EmotionClassificationNet(dropRate=0)


    def forward(self, x, test_mode=True):
        out = F.softmax(self.cnn1(x), dim=1)
        out += F.softmax(self.cnn2(x), dim=1)
        out += F.softmax(self.cnn3(x), dim=1)
        out += F.softmax(self.cnn4(x), dim=1)
        out += F.softmax(self.cnn5(x), dim=1)
        out /= 5
        
        if test_mode:
            return torch.argmax(out, dim=1)

        return out