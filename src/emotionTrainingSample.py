#!/usr/bin/env python3
import torch
import torch.nn as nn
import argparse
from tqdm.auto import tqdm
import matplotlib.pyplot as plt
#
# Parse the input arguments.
def getInputArgs():
    parser = argparse.ArgumentParser('Sample for training an emotion classification model.')
    parser.add_argument('--gpu', dest='gpu', default=torch.cuda.is_available(), type=bool, help='Use gpu for training')
    parser.add_argument('--nepoch', dest='nepoch', default=50, type=int, help='Number of training epochs')
    parser.add_argument('--mdl', dest='mdl', default=None, type=str, help='Model to load')
    parser.add_argument('--val', dest='val', action='store_true', help='Get validation score')
    args = parser.parse_args()
    return args
class EmotionClassificationNet(nn.Module):

    def __init__(self):
        super(EmotionClassificationNet, self).__init__()
        self.cnn = nn.Sequential(
            nn.Conv2d(1, 64, 3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.MaxPool2d(2, 2),
            nn.Dropout(0.25),

            nn.Conv2d(64, 128, 3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(128),
            nn.MaxPool2d(2, 2),
            nn.Dropout(0.25),

            nn.Conv2d(128, 128, 3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(128),
            nn.MaxPool2d(2, 2),
            nn.Dropout(0.25),

            nn.Conv2d(128, 128, 3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(128),
            nn.MaxPool2d(2, 2),
            nn.Dropout(0.25),

            nn.Conv2d(128, 128, 3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(128),
            nn.Dropout(0.25),
        )
        self.nn = nn.Sequential(
            nn.Linear(1152, 512),
            nn.ReLU(),
            nn.Dropout(0.25),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Dropout(0.25),
            nn.Linear(256, 7)

        )

    def forward(self, x, test_mode=False):
        batch_size = x.shape[0]
        feats = self.cnn(x)
        out = self.nn(feats.view(batch_size, -1))
        #
        # If we are testing then return prediction index.
        if test_mode:
            _, out = torch.max(out, 1)
        return out

def getDataset(args):
    import pathlib
    if pathlib.Path('./train_split.pth').exists():
        train_imgs, train_labels = torch.load('train_split.pth')
        probs = torch.ones(train_imgs.shape[0]) * 0.3
        val_set_mask = torch.bernoulli(probs).bool()
        val_imgs = train_imgs[val_set_mask]
        val_labels = train_labels[val_set_mask]
        train_imgs = train_imgs[~val_set_mask]
        train_labels = train_labels[~val_set_mask]
        return (train_imgs, train_labels), (val_imgs, val_labels)
    else:
        print('The provided dataset does not exist!')
        exit(0)

def getDataloader(args):
    train, val = getDataset(args)
    train_dataset = torch.utils.data.TensorDataset(*train)
    val_dataset = torch.utils.data.TensorDataset(*val)
    #
    # Due to class imbalance introduce a weighted random sampler to select rare classes more often.
    batch_size = 128
    weights_label = train[1].unique(return_counts=True, sorted=True)[1].float().reciprocal()
    weights = torch.zeros_like(train[1], dtype=torch.float)
    for idx, label in enumerate(train[1]):
        weights[idx] = weights_label[label]
    sampler = torch.utils.data.sampler.WeightedRandomSampler(weights, len(weights))
    #
    # Create the dataloaders for the different datasets.
    train_loader = torch.utils.data.DataLoader(train_dataset, batch_size=batch_size,
                                            num_workers=2, sampler=sampler)
    val_loader = torch.utils.data.DataLoader(val_dataset, batch_size=batch_size,
                                            shuffle=False, num_workers=2)
    return train_loader, val_loader

def train_loop(mdl, loss_fn, optim, dl, device):
    print('')
    pbar = tqdm(dynamic_ncols=True, total=int(len(dl)))
    n_batch_loss = 50
    running_loss = 0
    for nex, ex in enumerate(dl):
        ims, labels, = ex
        ims = ims.to(device)
        labels = labels.to(device)
        #
        # Optimization.
        optim.zero_grad()
        outs = mdl(ims)
        loss = loss_fn(outs, labels)
        loss.backward(loss)
        optim.step()
        #
        # Statistics
        running_loss +=  loss.item()
        nex += 1
        if nex % n_batch_loss == 0:
            status = 'L: %.4f '%(loss / n_batch_loss)
            running_loss = 0
            pbar.set_description(status)
        pbar.update(1)
    pbar.close()
    return mdl

def calc_acc(mdl, dl, dl_type, device):
    print('')
    with torch.no_grad():
        pbar = tqdm(dynamic_ncols=True, total=int(len(dl)))
        total = 0
        ncorrect = 0
        for nex, ex in enumerate(dl):
            ims, labels, = ex
            ims = ims.to(device)
            labels = labels.to(device)
            predicted = mdl(ims, True)
            total += labels.size(0)
            ncorrect += (predicted == labels).sum().item()
            status = '%s ACC: %.4f '%(dl_type, float(ncorrect) / total)
            pbar.set_description(status)
            pbar.update(1)
    pbar.close()
    return float(ncorrect) / total

if __name__ == "__main__":
    args = getInputArgs()
    train_dl, val_dl = getDataloader(args)
    mdl = EmotionClassificationNet()
    ce_loss = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(mdl.parameters())
    device = torch.device('cpu')
    if args.gpu:
        device = torch.device('cuda:0')
    if args.mdl is not None:
        mdl.load_state_dict(torch.load(args.mdl))
    mdl = mdl.to(device)
    if args.val:
        print('Val ACC loop')
        mdl.train(False)
        val_acc = calc_acc(mdl, val_dl, 'val', device)
        print('VAL ACC: ', val_acc)
    else:
        #
        # Training loop.
        best_val = -float('inf')
        for epoch in range(args.nepoch):
            print('Train loop')
            mdl.train(True)
            mdl = train_loop(mdl, ce_loss, optimizer, train_dl, device)
            print('Train ACC loop')
            mdl.train(False)
            train_acc = calc_acc(mdl, train_dl, 'train', device)
            print('Val ACC loop')
            val_acc = calc_acc(mdl, val_dl, 'val', device)
            #
            # Early stopping.
            if val_acc > best_val:
                best_val = val_acc
                torch.save(mdl.state_dict(), 'mdl_best.pth')
