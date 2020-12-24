import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os
from PIL import Image

import torch
import torch.nn.functional as F


class Net(torch.nn.Module):
    def __init__(self, n_feature, n_hidden, n_output, n_c):
        super(Net, self).__init__()
        self.hidden = torch.nn.Linear(n_feature, n_hidden)   # hidden layer
        self.box = torch.nn.Linear(n_hidden, n_output-1)   # output layer
        self.logit = torch.nn.Linear(n_hidden, 1)
        
        self.conv1 = torch.nn.Sequential(         # input shape (3, 80, 60)
            torch.nn.Conv2d(
                in_channels = n_c,            # input height
                out_channels = 8,             # n_filters
                kernel_size = 5,              # filter size
                stride = 2,                   # filter movement/step
                padding = 0,                  
            ),                              
            torch.nn.ReLU(),                      # activation
            #torch.nn.MaxPool2d(kernel_size = 2),    
        )
        self.conv2 = torch.nn.Sequential(       
            torch.nn.Conv2d(in_channels = 8, 
                            out_channels = 16, 
                            kernel_size = 5, 
                            stride = 2, 
                            padding = 0),      
            torch.nn.ReLU(),                      # activation
            #torch.nn.MaxPool2d(2),                
        )
        
        self.conv3 = torch.nn.Sequential(       
            torch.nn.Conv2d(in_channels = 16, 
                            out_channels = 8, 
                            kernel_size = 1, 
                            stride = 1, 
                            padding = 0),      
            torch.nn.ReLU(),                      # activation
            #torch.nn.MaxPool2d(2),                
        )
    def forward(self, x):
        feat = self.conv1(x)
        feat = self.conv2(feat)
        feat = self.conv3(feat)
        feat = feat.view(feat.size(0), -1)
        x2 = F.relu(self.hidden(feat))      # activation function for hidden layer
        
        out_box = F.relu(self.box(x2))            # linear output
        out_logit = torch.sigmoid(self.logit(x2))
        
        return out_box, out_logit
        
class Detector(object):
    """docstring for Detector"""
    def __init__(self):
        super(Detector, self).__init__()
        # TODO: MEAN & STD
        self.mean = [[[[0.5548078,  0.56693329, 0.53457436]]]] 
        self.std = [[[[0.26367019, 0.26617227, 0.25692861]]]]
        self.img_size = 100 
        self.img_size_w = 80
        self.img_size_h = 60
        self.min_object_size = 10
        self.max_object_size = 40 
        self.num_objects = 1
        self.num_channels = 3
        self.model = Net(n_feature = 1632, n_hidden = 128, n_output = 5, n_c = 3)     # define the network


    def load(self, PATH):
        # self.model = torch.load(PATH)
        # self.model.eval()

        self.model.load_state_dict(torch.load(PATH))
        self.model.eval()

    def forward(self, img, downscale=1):   
        ##Add a dimension
        img = np.expand_dims(img.transpose(1,0,2), 0) / 255

        ch1 = img[:,:,:,0].copy()
        ch3 = img[:,:,:,2].copy()

        img[:,:,:,0] = ch3
        img[:,:,:,2] = ch1

        ##Preprocess
        img = (img - self.mean)/self.std

        ##Transpose to model format
        if(img.shape[1] != self.num_channels):
            img = img.transpose((0,3,1,2))

        ##Detect
        with torch.no_grad():
            pred_y_box, pred_y_logit = self.model.forward(torch.tensor(img, dtype=torch.float32))

            pred_y_box, pred_y_logit = pred_y_box.numpy(), pred_y_logit.numpy()
            pred_y_label = pred_y_logit > 0.5
            pred_bboxes = pred_y_box * self.img_size
            # pred_bboxes = pred_bboxes.reshape(len(pred_bboxes), num_objects, -1)

        return pred_bboxes, pred_y_label

