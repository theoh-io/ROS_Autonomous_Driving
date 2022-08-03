import torch
import numpy as np
import time


class YoloDetector():
    def __init__(self, model='default', verbose = False):
        yolo_version='yolov5s'
        if model=='default':
            self.model = torch.hub.load('ultralytics/yolov5', yolo_version)
            print(f"-> Using {yolo_version} for multi bbox detection.")
        self.model.classes=0 #running only person detection
        self.detection=np.array([0, 0, 0, 0])
        self.verbose=verbose
        print(f"Created YOLO detector with verbose={verbose}.")

    def bbox_format(self):
        #detection format xmin, ymin, xmax,ymax, conf, class, 'person'
        #bbox format xcenter, ycenter, width, height
        if(self.detection.shape[0]==1):
          self.detection=np.squeeze(self.detection)
          xmin=self.detection[0]
          ymin=self.detection[1]
          xmax=self.detection[2]
          ymax=self.detection[3]
          x_center=(xmin+xmax)/2
          y_center=(ymin+ymax)/2
          width=xmax-xmin
          height=ymax-ymin
          bbox=[x_center, y_center, width, height]
          bbox=np.expand_dims(bbox, axis=0)
          return bbox
        else:
          bbox_list=[]
          for i in range(self.detection.shape[0]):
            xmin=self.detection[i][0]
            ymin=self.detection[i][1]
            xmax=self.detection[i][2]
            ymax=self.detection[i][3]
            x_center=(xmin+xmax)/2
            y_center=(ymin+ymax)/2
            width=xmax-xmin
            height=ymax-ymin
            bbox_unit=np.array([x_center, y_center, width, height])
            bbox_list.append(bbox_unit)
          bbox_list=np.vstack(bbox_list)
          #bbox_list=bbox_list.tolist()
          return bbox_list

            

    def best_detection(self):
        #tensor dim is now (number of detections, 7)
        #output dim is (1,7)
        N=self.detection.shape[0]
        if(N != 1):
            if self.verbose is True: print("multiple persons detected")
            #extracting the detection with max confidence
            idx=np.argmax(self.detection[range(N),4])
            self.detection=np.expand_dims(self.detection[idx], 0)
        #else: #1 detection
            #self.detection=np.squeeze(self.detection)


    def forward(self, image, downscale, thresh=0.01):
        #threshold for confidence detection
        # Inference
        tic = time.perf_counter()
        results = self.model(image) #might need to specify the size
        toc = time.perf_counter()
        if self.verbose is True: print(f"Elapsed time for yolov5 inference: {(toc-tic)*1e3:.1f}ms")
        #results.xyxy: [xmin, ymin, xmax, ymax, conf, class]
        detect_pandas=results.pandas().xyxy
        self.detection=np.array(detect_pandas)
        if self.verbose is True: print("shape of the detection: ", self.detection.shape)
        if (self.detection.shape[1]!=0):
            #print("DETECTED SOMETHING !")
            #use np.squeeze to remove 0 dim from the tensor
            self.detection=np.squeeze(self.detection,axis=0) 

            #class function to decide which detection to keep
            self.best_detection()
            if(self.detection[0][4]<thresh):
                print("detection under thresh")
                return [[0.0, 0.0, 0.0, 0.0]], [[False]], [[0.0, 0.0, 0.0, 0.0]]
            #modify the format of detection for bbox
            bbox=self.bbox_format()
            #bbox=self.detection
            #if self.verbose is True: print("bbox after format: ", bbox)
            return bbox, [[True]], [[0.0, 0.0, 0.0, 0.0]]
        return [[0.0, 0.0, 0.0, 0.0]], [[False]], [[0.0, 0.0, 0.0, 0.0]]

    def predict(self, image, thresh=0.01):
        # Inference
        tic = time.perf_counter()
        results = self.model(image) #might need to specify the size
        toc = time.perf_counter()
        if self.verbose is True: print(f"Elapsed time for yolov5 inference: {(toc-tic)*1e3:.1f}ms")
        detect_pandas=results.pandas().xyxy  #results.xyxy: [xmin, ymin, xmax, ymax, conf, class]
        self.detection=np.array(detect_pandas)
        if self.verbose is True: print("shape of the detection: ", self.detection.shape)
        if (self.detection.shape[1]!=0):
            #save resuts
            #results.save()
            self.detection=np.squeeze(self.detection,axis=0)   #use np.squeeze to remove 0 dim from the tensor
            #if self.verbose is True: print("bbox before format: ", self.detection)
            bbox=self.bbox_format() #modify the format of detection for [xcenter, ycenter, width, height]
            if self.verbose is True: print("bbox after format: ", bbox)
            return bbox
        return None