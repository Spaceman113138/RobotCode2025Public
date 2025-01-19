import queue
from typing import List, Tuple, Union

import cv2

from config.config import ConfigStore
from output.StreamServer import MjpegServer
from output.overlay_util import overlay_obj_detect_observation
from pipeline.ObjectDetector import CoreMLObjectDetector
from vision_types import ObjDetectObservation

def objdetect_worker(
        q_in: queue.Queue[Tuple[float, cv2.Mat, ConfigStore]],
        q_out: queue.Queue[Tuple[float, List[ObjDetectObservation]]],
        server_port: int):
    object_detector = CoreMLObjectDetector()
    stream_server = MjpegServer()
    stream_server.start(server_port)

    while True:
        sample = q_in.get()
        timestamp: float = sample[0]
        image: cv2.Mat = sample[1]
        config: ConfigStore = sample[2]

        observations = object_detector.detect(image, config)
        [overlay_obj_detect_observation(image, x) for x in observations]

        q_out.put((timestamp, observations))
        stream_server.set_frame(image)
        