import cv2
from datetime import datetime
import subprocess
import queue
import threading

from config.config import ConfigStore

FRAMERATE = 25

class VideoWriter:
    def __init__(self) -> None:
        raise NotImplementedError
    
    def start(self, config: ConfigStore) -> None: 
        raise NotImplementedError
    
    def stop(self) -> None:
        raise NotImplementedError
    
    def write_frame(self, timestamp: float, frame: cv2.Mat) -> None:
        raise NotImplementedError
    

class FFmpegVideoWriter(VideoWriter):
    def __init__(self) -> None:
        pass

    def start(self, config: ConfigStore) -> None:
        filename = config.local_config.video_folder + config.local_config.device_id + "_" + datetime.fromtimestamp(config.remote_config.timestamp).strftime("%Y%m%d_%H%M%S") + ".mkv"
        self._ffmpeg = subprocess.Popen(["ffmpeg", "-y", "-s", str(config.remote_config.camera_resolution_width) + "x" + str(config.remote_config.camera_resolution_height), "-pixel_format", "bgr24", "-r", str(FRAMERATE), "-re", "-f", "rawvideo", "-i", "pipe:", "-c:v", "hevc_videotoolbox", "-q:v", "50", "-pix_fmt", "yuv420p", "-vf", "setpts=PTS-STARTPTS", filename], stdin=subprocess.PIPE)
        
        self._running = True
        self._queue = queue.Queue(maxsize=1)
        self._thread = threading.Thread(target=self._frame_thread, args=(self._queue,), daemon=True)
        self._thread.start()
        
    def stop(self) -> None:
        self._running = False
        self._thread.join()
        self._ffmpeg.kill()

    def write_frame(self, timestamp: float, frame: cv2.Mat) -> None:
        try:
            self._queue.put(frame, block=False)
        except:
            pass

    def _frame_thread(self, q_in: queue.Queue[cv2.Mat]) -> None: 
        while self._running: 
            frame = q_in.get()
            self._ffmpeg.stdin.write(frame.tobytes())
    