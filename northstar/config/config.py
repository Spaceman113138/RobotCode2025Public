from dataclasses import dataclass
import numpy
import numpy.typing


@dataclass
class LocalConfig:
    device_id: str = ""
    server_ip: str = ""
    apriltags_stream_port: int = 8000
    objdetect_stream_port: int = 8001
    capture_impl: str = ""
    obj_detect_model: str = ""
    apriltags_enable: bool = False
    objdetect_enable: bool = True
    video_folder: str = ""
    has_calibration: bool = False
    camera_matrix: numpy.typing.NDArray[numpy.float64] = None
    distortion_coefficients: numpy.typing.NDArray[numpy.float64] = None


@dataclass
class RemoteConfig:
    camera_id: str = ""
    camera_resolution_width: int = 0
    camera_resolution_height: int = 0
    camera_auto_exposure: int = 0
    camera_exposure: int = 0
    camera_gain: float = 0
    fiducial_size_m: float = 0
    tag_layout: any = None
    is_recording: bool = False
    timestamp: int = 0



@dataclass
class ConfigStore:
    local_config: LocalConfig
    remote_config: RemoteConfig
