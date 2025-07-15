from typing import Dict
from depthai import Pipeline
import depthai as dai
from depthai import MonoCameraProperties, ColorCameraProperties
from abc import ABC, abstractmethod


class DepthAiPipeline(ABC):
    """
    Stable base class for DepthAI Pipelines
    """

    def __init__(self) -> None:
        """
        Initialize pipeline with stable settings
        """
        self.pipeline = None
        self.pipeline = Pipeline()
        
        self.configure()

    @abstractmethod
    def configure(self):
        """
        Configure the pipeline - all children must implement
        """
        pass

    @abstractmethod
    def get_output_queues(self, device: dai.Device) -> Dict[str, dai.DataOutputQueue]:
        """
        Get output queues - all children must implement
        """
        pass

    def make_mono_camera(self, name: str) -> dai.node.MonoCamera:
        """
        Build stable mono camera
        """
        camera = self.pipeline.create(dai.node.MonoCamera)
        
        camera.setResolution(MonoCameraProperties.SensorResolution.THE_800_P)
        camera.setFps(30)
        camera.setCamera(name)
        
        return camera

    def make_color_camera(self) -> dai.node.ColorCamera:
        """
        Build stable color camera
        """
        camera = self.pipeline.create(dai.node.ColorCamera)
        camera.setResolution(ColorCameraProperties.SensorResolution.THE_1080_P)
        camera.setInterleaved(False)
        camera.setBoardSocket(dai.CameraBoardSocket.RGB)
        camera.setFps(30)
        
        camera.initialControl.setManualFocus(130)
        camera.initialControl.setManualExposure(8000, 400)

        return camera

    def make_stereo_sensor(self) -> dai.node.StereoDepth:
        """
        Build stereo sensor with stable settings
        """
        stereo = self.pipeline.create(dai.node.StereoDepth)
        
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        
        stereo.setExtendedDisparity(False)
        stereo.setRectifyEdgeFillColor(0)
        
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)
        
        stereo.initialConfig.setConfidenceThreshold(230)
        
        return stereo

    def make_image_manip(self, resize_dim: int, keep_aspect: bool) -> dai.node.ImageManip:
        """
        Build stable ImageManip node
        """
        manip = self.pipeline.create(dai.node.ImageManip)
        manip.initialConfig.setResize(resize_dim, resize_dim)
        manip.initialConfig.setFrameType(dai.RawImgFrame.Type.RGB888p)
        
        manip.setKeepAspectRatio(keep_aspect)
        
        manip.setMaxOutputFrameSize(resize_dim * resize_dim * 3)
        manip.setWaitForConfigInput(False)
        manip.setNumFramesPool(3)
        
        return manip

    def make_net(
        self,
        network: type[dai.node.DetectionNetwork],
        blob_path: str,
        threshold: float,
        queue_size: int,
        blocking: bool,
    ) -> dai.node.DetectionNetwork:
        """
        Build stable neural network
        """
        try:
            net = self.pipeline.create(network)
            net.setConfidenceThreshold(threshold)
            net.setBlobPath(blob_path)
            
            net.input.setQueueSize(queue_size)
            net.input.setBlocking(blocking)
            net.setNumInferenceThreads(2)
            
            return net
            
        except Exception as e:
            raise

    def make_output(self, node_output: dai.Node.Output, name: str) -> dai.node.XLinkOut:
        """
        Build stable XLinkOut node
        """
        output = self.pipeline.create(dai.node.XLinkOut)
        output.setStreamName(name)
        
        output.input.setBlocking(False)
        output.input.setQueueSize(4)

        node_output.link(output.input)

        return output

    def enable_laser_projector(self, device: dai.Device, power: int = 800):
        """
        Enable laser dot projector
        """
        try:
            device.setIrLaserDotProjectorBrightness(power)
            
            if hasattr(device, 'setIrFloodLightBrightness'):
                device.setIrFloodLightBrightness(0)
            
            return True
            
        except Exception as e:
            return False

    def optimize_device_settings(self, device: dai.Device):
        """
        Apply stable device settings
        """
        try:
            if hasattr(device, 'setLogLevel'):
                device.setLogLevel(dai.LogLevel.WARN)
            if hasattr(device, 'setLogOutputLevel'):
                device.setLogOutputLevel(dai.LogLevel.WARN)
            
        except Exception as e:
            pass

    def get_device_info(self, device: dai.Device) -> Dict:
        """
        Get device information
        """
        try:
            info = {
                'device_name': device.getDeviceName(),
                'usb_speed': str(device.getUsbSpeed()),
                'cameras': list(device.getConnectedCameras()),
                'laser_projector': False,
                'firmware_version': device.getBootloaderVersion()
            }
            
            try:
                device.setIrLaserDotProjectorBrightness(0)
                info['laser_projector'] = True
            except:
                info['laser_projector'] = False
                
            return info
            
        except Exception as e:
            return {'error': str(e)}

    def validate_camera_setup(self, device: dai.Device) -> bool:
        """
        Validate camera setup
        """
        try:
            cameras = device.getConnectedCameras()
            
            has_left = dai.CameraBoardSocket.LEFT in cameras
            has_right = dai.CameraBoardSocket.RIGHT in cameras
            
            if not (has_left and has_right):
                return False
            
            laser_available = self.enable_laser_projector(device, 0)
            
            return True
            
        except Exception as e:
            return False