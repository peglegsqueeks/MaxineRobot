from ..depth_ai.CameraReading import CameraReading
from ..depth_ai.ObjectDetectionReading import ObjectDetectionReading
from ..types.CameraMode import CameraMode
from ..depth_ai.DepthAiPipeline import DepthAiPipeline
from ..depth_ai.ObjectDetectionPipeline import ObjectDetectionPipeline
from ..sensors.RobotSensor import RobotSensor
import depthai as dai
import time


class CameraSensor(RobotSensor):
    """
    Stable camera sensor for reliable person detection
    """

    MODE_PIPELINES = {CameraMode.OBJECT_DETECTION: ObjectDetectionPipeline()}
    MODE_READINGS = {CameraMode.OBJECT_DETECTION: ObjectDetectionReading}

    def __init__(self, starting_mode: CameraMode = CameraMode.DISABLED) -> None:
        """
        Initialize stable camera
        """
        super().__init__("Stable Camera")
        self.device = None
        self.current_mode = CameraMode.DISABLED
        self.output_queues = {}
        self.camera_info = {}
        
        self.laser_projector_enabled = False
        
        self.switch_mode(starting_mode)

    def stop_camera(self):
        """
        Stop camera
        """
        if self.device is not None:
            try:
                if self.laser_projector_enabled:
                    self.device.setIrLaserDotProjectorBrightness(0)
                    self.laser_projector_enabled = False
                
                self.device.close()
            except Exception as e:
                pass
            finally:
                self.device = None
                self.output_queues = {}
                self.camera_info = {}

    def start_camera(self, pipeline: DepthAiPipeline):
        """
        Start camera with stable settings
        """
        if self.device is not None:
            raise Exception("Camera already started - call stop_camera() first")

        try:
            self.device = dai.Device()
            
            self.camera_info = pipeline.get_device_info(self.device)
            
            if not pipeline.validate_camera_setup(self.device):
                raise Exception("Camera setup validation failed")
            
            self.device.startPipeline(pipeline.pipeline)
            
            self.enable_laser_projector()
            
        except Exception as e:
            self.device = None
            self.camera_info = {}
            raise

    def enable_laser_projector(self, power: int = 800):
        """
        Enable laser dot projector
        """
        try:
            if not self.device:
                return False
            
            self.device.setIrLaserDotProjectorBrightness(power)
            
            if hasattr(self.device, 'setIrFloodLightBrightness'):
                self.device.setIrFloodLightBrightness(0)
            
            self.laser_projector_enabled = True
            return True
            
        except Exception as e:
            self.laser_projector_enabled = False
            return False

    def switch_mode(self, mode: CameraMode):
        """
        Switch camera mode
        """
        self.stop_camera()

        if mode == CameraMode.DISABLED:
            self.current_mode = CameraMode.DISABLED
            return

        if mode not in self.MODE_PIPELINES:
            self.current_mode = CameraMode.DISABLED
            return

        pipeline = self.MODE_PIPELINES[mode]
        try:
            self.start_camera(pipeline)
            self.current_mode = mode
            self.output_queues = pipeline.get_output_queues(self.device)
            
        except Exception as e:
            self.current_mode = CameraMode.DISABLED
            self.output_queues = {}

    def get_reading(self) -> CameraReading:
        """
        Get stable camera reading
        """
        if (self.current_mode == CameraMode.DISABLED or 
            not self.device or 
            not self.output_queues):
            return None

        readings = {}
        for name, queue in self.output_queues.items():
            try:
                latest = queue.get() if queue.has() else None
                readings[name] = latest
            except Exception as e:
                readings[name] = None

        try:
            if self.current_mode not in self.MODE_READINGS:
                return None

            reading = self.MODE_READINGS[self.current_mode](readings)
            return reading
            
        except Exception as e:
            return None

    def get_closest_person_fast(self) -> dict:
        """
        Get closest person detection with stabilized position handling
        """
        reading = self.get_reading()
        
        if not reading or not hasattr(reading, 'get_closest_person_with_depth'):
            return None
        
        # Get the detection with stabilized position data
        result = reading.get_closest_person_with_depth()
        
        if result and hasattr(reading, 'get_stabilized_position'):
            # Ensure we have the most up-to-date stabilized position
            people = reading.get_people_locations()
            if people:
                stabilized_data = reading.get_stabilized_position(people[0])
                if stabilized_data:
                    result['bbox_center'] = stabilized_data
                    result['detection_quality'] = stabilized_data.get('detection_quality', 'unknown')
                    result['is_occluded'] = stabilized_data.get('detection_quality') == 'occluded'
        
        return result

    def get_camera_info(self) -> dict:
        """
        Get camera information
        """
        info = self.camera_info.copy() if self.camera_info else {}
        
        info.update({
            'current_mode': self.current_mode.value if hasattr(self.current_mode, 'value') else str(self.current_mode),
            'laser_projector_enabled': self.laser_projector_enabled,
            'device_connected': self.device is not None
        })
        
        return info

    def get_queue_status(self) -> dict:
        """
        Get queue status for debugging
        """
        if not self.output_queues:
            return {}
        
        status = {}
        for name, queue in self.output_queues.items():
            try:
                queue_name = name.value if hasattr(name, 'value') else str(name)
                status[queue_name] = {
                    'has_data': queue.has() if queue else False,
                    'queue_size': queue.size() if queue and hasattr(queue, 'size') else 0,
                    'available': queue is not None
                }
            except Exception as e:
                queue_name = name.value if hasattr(name, 'value') else str(name)
                status[queue_name] = {
                    'has_data': False,
                    'queue_size': 0,
                    'available': False,
                    'error': str(e)
                }
        
        return status

    def get_performance_stats(self) -> dict:
        """
        Get performance statistics
        """
        return {
            'mode': str(self.current_mode),
            'laser_projector': self.laser_projector_enabled,
            'device_connected': self.device is not None,
            'queue_count': len(self.output_queues)
        }

    def force_laser_projector_power(self, power: int):
        """
        Force set laser projector power
        """
        if self.device:
            try:
                self.device.setIrLaserDotProjectorBrightness(power)
                self.laser_projector_enabled = power > 0
                return True
            except Exception as e:
                return False
        return False