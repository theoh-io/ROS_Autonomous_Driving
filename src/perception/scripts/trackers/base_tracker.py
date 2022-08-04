class BaseTracker():
    def __init__(self, tracker_name, tracking_conf, device='gpu'):
        self.tracker_name=tracker_name
        self.conf=tracking_conf
        self.device=device

    def forward(self, cut_imgs, detections, img):
        raise NotImplementedError("Tracker Base Class does not provide a forward method.")

