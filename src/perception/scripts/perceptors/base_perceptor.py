import ..detectors.yolov5_detector

class BasePerceptor():
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.initialize()

    def initialize(self, width=640, height=480, channels=3, downscale=4, 
                    detector=yolov5_detector.Yolov5Detector(), tracker=None, 
                    type_input="opencv", detector_size="default"):
        # perceptor expected input image dimensions
        self.width = int(width/downscale)
        self.height = int(height/downscale)
        self.downscale = downscale

        # Image received size data.
        self.data_size = int(self.width * self.height * channels)

        self.detector=detector(detector_size, verbose=False)
        self.tracker=tracker
        self.type_input=type_input

        if self.verbose:
            print("Initializing Perceptor")
            print(f"-> Using {self.detector} for detection and {self.tracker} 
                    for tracking")
            print(f"-> Input image of type {self.type_input} and downscale {self.downscale}")


    def forward(self, image):
        raise NotImplementedError("perceptor Base Class does not provide a forward method.")


