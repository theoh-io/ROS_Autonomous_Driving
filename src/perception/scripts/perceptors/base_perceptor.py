from detectors.yolov5_detector import Yolov5Detector
from trackers.mmtracking_sot import SotaTracker


class BasePerceptor():

    def __init__(self, width=640, height=480, channels=3, downscale=4, 
                    detector=Yolov5Detector(), detector_size="default",
                    tracker=None, tracker_model=None, tracking_conf=0.5,
                    type_input="opencv", verbose="False"):
        # perceptor expected input image dimensions
        self.width = int(width/downscale)
        self.height = int(height/downscale)
        self.downscale = downscale
        self.verbose=verbose

        # Image received size data.
        self.data_size = int(self.width * self.height * channels)

        self.detector=detector(detector_size, verbose=self.verbose)
        if tracker:
            self.tracker=tracker(tracker_model, tracking_conf, verbose=self.verbose)
        self.type_input=type_input

        if self.verbose:
            print("Initializing Perceptor")
            #print(f"-> Using {str(self.detector)} for detection and {str(self.tracker)} 
            #        for tracking")
            print(f"-> Input image of type {self.type_input} and downscale {self.downscale}")


    def forward(self, image):
        raise NotImplementedError("perceptor Base Class does not provide a forward method.")


