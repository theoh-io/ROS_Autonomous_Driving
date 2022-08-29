import cv2
import numpy as np
from PIL import Image
from perceptors.base_perceptor import BasePerceptor
from tools.utils import Utils


class MotPerceptor(BasePerceptor):
    # Initialize detector and its main properties
    # def __init__(self, width, height, channels, downscale, global_path='', detector='', load=True, type_input="opencv", save_video=False, filename_video=""):
    #     # Detector expected input image dimensions
    #     self.width = int(width/downscale)
    #     self.height = int(height/downscale)
    #     self.downscale = downscale
    #     # Image received size data.
    #     self.data_size = int(self.width * self.height * channels)
    #     self.global_path = global_path
    #     self.detector = detector
    #     self.type_input = type_input
    #     if load:
    #         self.detector.load(global_path)

    def forward(self, received_image):
        # Adapt image to detector requirements
        # pil_image = Image.frombytes('RGB', (self.width,self.height), received_image)
        # opencvImage = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
        # opencvImage = cv2.cvtColor(opencvImage,cv2.COLOR_BGR2RGB)
        image=self.preproc(received_image)

        bbox_list, bbox_label , bbox_legs= self.detector.forward(image, self.downscale)

        for bbox in bbox_list:
            Utils.bbox_vis(bbox, self.opencvImage)  

        return bbox_list, bbox_label, bbox_legs, self.opencvImage

