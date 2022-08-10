import torch
import numpy as np

#Yolo Inference Settings
#model.conf = 0.25  # NMS confidence threshold
#      iou = 0.45  # NMS IoU threshold
#      agnostic = False  # NMS class-agnostic
#      multi_label = False  # NMS multiple labels per box
#      classes = None  # (optional list) filter by class, i.e. = [0, 15, 16] for COCO persons, cats and dogs
#      max_det = 1000  # maximum number of detections per image
#      amp = False  # Automatic Mixed Precision (AMP) inference
#results = model(imgs, size=320)  # custom inference size


class Yolov5Detector():
    def __init__(self, model_size='default', verbose = False):       
        if model_size=='default':
            self.yolo_version = "yolov5s"
        elif model_size=="medium":
            self.yolo_version = "yolov5m"
        elif model_size=="large":
            self.yolo_version = "yolov5l"
        else:
            print("unable to find detector model, loading YOLOv5s")
            self.yolo_version = "yolov5s"
        self.model = torch.hub.load('ultralytics/yolov5', self.yolo_version)

        #Detector attributes
        self.verbose = verbose
        self.model.classes=0 #running only person detection
        self.detection=np.array([0, 0, 0, 0])

        if self.verbose: 
            print(f"-> Using {self.yolo_version} for detection.")
        

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
          if self.verbose is True: print(self.detection.shape)
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
            if self.verbose is True: print(bbox_unit)
            bbox_list.append(bbox_unit)
          bbox_list=np.vstack(bbox_list)
          #bbox_list=bbox_list.tolist()
          if self.verbose is True: print("final bbox", bbox_list)
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


    def unique_predict(self, image, thresh=0.01):
        #threshold for confidence detection
        # Inference
        results = self.model(image) #might need to specify the size

        #results.xyxy: [xmin, ymin, xmax, ymax, conf, class]
        detect_pandas=results.pandas().xyxy
        self.detection=np.array(detect_pandas)
        if self.verbose is True: print(self.detection)
        if (self.detection.shape[1]!=0):
            #print("DETECTED SOMETHING !")
            #use np.squeeze to remove 0 dim from the tensor
            self.detection=np.squeeze(self.detection,axis=0) 

            #class function to decide which detection to keep
            self.best_detection()
            if(self.detection[0][4]>thresh):
                label=True
            #modify the format of detection for bbox
            bbox=self.bbox_format()
            return bbox, label
        return None,False

    def forward(self, image, thresh=0.01):
        #threshold for confidence detection
        # Inference
        results = self.model(image) #might need to specify the size

        #results.xyxy: [xmin, ymin, xmax, ymax, conf, class]
        detect_pandas=results.pandas().xyxy

        self.detection=np.array(detect_pandas)
        if self.verbose is True: print("shape of the detection: ", self.detection.shape)
        #print("detection: ",self.detection)

        if (self.detection.shape[1]!=0):
            if self.verbose is True: print("DETECTED SOMETHING !!!")
            #save resuts
            #results.save()
            
            #use np.squeeze to remove 0 dim from the tensor
            self.detection=np.squeeze(self.detection,axis=0) 
            if self.verbose is True: print("bbox before format: ", self.detection)
            #modify the format of detection for bbox
            bbox=self.bbox_format()
            if self.verbose is True: print("bbox after format: ", bbox)
            return bbox
        return None