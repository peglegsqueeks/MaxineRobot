#!/usr/bin/env python3
"""
WORKING OptimizedLidarTestMode.py + COORDINATE VERIFICATION
EXACTLY THE SAME as working version + coordinate verification metrics
NO CHANGES to head tracking system - only added coordinate verification

File: src/behaviors/lidar_test_mode/CoordinateVerificationLidarTest.py
"""
from __future__ import annotations

import pygame
import math
import time
import threading
import queue
import py_trees
import csv
import os
import statistics
import numpy as np
import cv2
from collections import deque
from datetime import datetime
from py_trees.common import Status
from pyrplidar import PyRPlidar

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.types.RobotModes import RobotMode
from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig

try:
    import depthai as dai
except ImportError:
    dai = None
    print("⚠️ DepthAI not available - will use fallback detection")

# ======================================================================================
# Small math helpers (UNCHANGED from working version)
# ======================================================================================

def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def _median(vals):
    s = sorted(v for v in vals if isinstance(v, (int, float)) and math.isfinite(v))
    if not s:
        return float("nan")
    n = len(s)
    m = n // 2
    return s[m] if n % 2 else 0.5 * (s[m - 1] + s[m])

# ======================================================================================
# OneEuro filter (UNCHANGED from working version)
# ======================================================================================

class _LowPass:
    def __init__(self):
        self.y = None
    def reset(self):
        self.y = None
    def filt(self, x: float, alpha: float) -> float:
        if self.y is None:
            self.y = x
        else:
            self.y = (1 - alpha) * self.y + alpha * x
        return self.y

class _OneEuro:
    def __init__(self, min_cutoff: float = 1.5, beta: float = 2.0, d_cutoff: float = 1.0):
        # cutoff/Hz tuned for snappy tracking
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.dx = _LowPass()
        self.x = _LowPass()
        self._last_t: float | None = None
    @staticmethod
    def _alpha(cutoff_hz: float, dt: float) -> float:
        # tau = 1/(2*pi*fc); alpha = 1/(1+tau/dt)
        tau = 1.0 / (2.0 * math.pi * max(1e-6, cutoff_hz))
        return 1.0 / (1.0 + tau / max(1e-6, dt))
    def reset(self):
        self.dx.reset(); self.x.reset(); self._last_t = None
    def filt(self, t: float, x: float) -> float:
        if self._last_t is None:
            self._last_t = t
            return self.x.filt(x, 1.0)
        dt = max(1e-3, t - self._last_t)
        self._last_t = t
        # derivative
        a_d = self._alpha(self.d_cutoff, dt)
        dx = self.dx.filt((x - (self.x.y if self.x.y is not None else x)) / dt, a_d)
        # dynamic cutoff increases with motion speed
        cutoff = self.min_cutoff + self.beta * abs(dx)
        a = self._alpha(cutoff, dt)
        return self.x.filt(x, a)

# ======================================================================================
# Head command filter — FASTER version (UNCHANGED from working version)
# ======================================================================================

class _HeadFilterCfg:
    def __init__(self):
        # FASTER OneEuro parameters (only change from original)
        self.euro_min_cutoff = 3.0     # Hz - FASTER (was 1.6)
        self.euro_beta = 1.8           # FASTER response (was 2.2)
        self.euro_dcut = 2.0           # Hz - FASTER (was 1.0)
        # Steadiness (unchanged from original)
        self.deadband_rad = math.radians(0.25)   
        self.hysteresis_rad = math.radians(0.3)
        # FASTER Limits (increased from original)
        self.max_yaw_rate = math.radians(480)    # deg/s - FASTER (was 320)
        self.max_pitch_rate = math.radians(200)  # FASTER (was 140)
        self.max_yaw_accel = math.radians(2400)  # deg/s^2 - FASTER (was 1600)
        self.max_pitch_accel = math.radians(1200) # FASTER (was 900)
        self.yaw_min = math.radians(-90)
        self.yaw_max = math.radians(90)
        self.pitch_min = math.radians(-30)
        self.pitch_max = math.radians(30)
        # Behavior (unchanged from original)
        self.det_timeout_s = 0.35
        self.lead_time_s = 0.14
        self.lock_rate_boost = 2.0
        self.catchup_err_rad = math.radians(12)
        self.rate_floor_rad_s = math.radians(18)

class _HeadCommandFilter:
    """Adaptive smoothing + acceleration-limited rate control with rate floor.
    Works only on final commands. Upstream pipelines unaffected.
    """
    def __init__(self, cfg: _HeadFilterCfg | None = None) -> None:
        self.cfg = cfg or _HeadFilterCfg()
        self._yaw_cmd = 0.0
        self._pitch_cmd = 0.0
        self._yaw_rate = 0.0
        self._pitch_rate = 0.0
        self._last_time: float | None = None
        self._last_det_ts: float = 0.0
        self._last_output: float | None = None
        self._euro = _OneEuro(self.cfg.euro_min_cutoff, self.cfg.euro_beta, self.cfg.euro_dcut)

    def note_person_lock(self, locked: bool) -> None:
        if locked:
            self._last_det_ts = time.monotonic()

    def _prefer_detection(self) -> bool:
        return (time.monotonic() - self._last_det_ts) <= self.cfg.det_timeout_s

    def _accel_rate_follow(self, desired_rate: float, current_rate: float, max_rate: float, max_accel: float, dt: float) -> float:
        desired_rate = _clamp(desired_rate, -max_rate, max_rate)
        delta = desired_rate - current_rate
        step = _clamp(delta, -max_accel * dt, max_accel * dt)
        return current_rate + step

    def update(self, yaw_target: float, pitch_target: float, person_locked: bool) -> tuple[float, float]:
        now = time.monotonic()
        if self._last_time is None:
            self._last_time = now
        dt = max(1e-3, now - self._last_time)
        self._last_time = now

        # Deadband: prevent dithering, but much smaller to avoid stall
        if abs(yaw_target - self._yaw_cmd) < self.cfg.deadband_rad:
            yaw_target = self._yaw_cmd
        if abs(pitch_target - self._pitch_cmd) < self.cfg.deadband_rad:
            pitch_target = self._pitch_cmd

        # Adaptive smoothing via OneEuro
        self.note_person_lock(person_locked)
        yaw_f = self._euro.filt(now, yaw_target)

        # Hysteresis only around zero crossing
        if self._last_output is not None:
            if (self._last_output < 0 < yaw_f and abs(yaw_f) < self.cfg.hysteresis_rad) or \
               (self._last_output > 0 > yaw_f and abs(yaw_f) < self.cfg.hysteresis_rad):
                yaw_f = 0.0

        # Lead prediction already applied upstream; still compute error here
        err = _wrap_pi(yaw_f - self._yaw_cmd)
        desired_yaw_rate = err / dt
        desired_pitch_rate = (pitch_target - self._pitch_cmd) / dt

        # Apply catchup if far from target
        max_yaw_rate = self.cfg.max_yaw_rate * (self.cfg.lock_rate_boost if self._prefer_detection() else 1.0)
        if abs(err) > self.cfg.catchup_err_rad:
            desired_yaw_rate = math.copysign(max_yaw_rate, err)

        # Rate floor to avoid stickslip near small errors
        if 0 < abs(err) < self.cfg.catchup_err_rad:
            desired_yaw_rate = math.copysign(max(abs(desired_yaw_rate), self.cfg.rate_floor_rad_s), err)

        # Accelerationlimited rate following
        self._yaw_rate = self._accel_rate_follow(desired_yaw_rate, self._yaw_rate, max_yaw_rate, self.cfg.max_yaw_accel, dt)
        self._pitch_rate = self._accel_rate_follow(desired_pitch_rate, self._pitch_rate, self.cfg.max_pitch_rate, self.cfg.max_pitch_accel, dt)

        # Integrate to commands
        yaw_cmd = _clamp(self._yaw_cmd + self._yaw_rate * dt, self.cfg.yaw_min, self.cfg.yaw_max)
        pitch_cmd = _clamp(self._pitch_cmd + self._pitch_rate * dt, self.cfg.pitch_min, self.cfg.pitch_max)

        self._yaw_cmd, self._pitch_cmd = yaw_cmd, pitch_cmd
        self._last_output = yaw_cmd
        return yaw_cmd, pitch_cmd

# ======================================================================================
# LiDAR device wrappers (UNCHANGED from working version)
# ======================================================================================

class UltraStablePyRPLidarA3:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, timeout=2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None
        self.motor_pwm = 600
        self.stability_mode = 4
        self.current_mode = None

    def connect(self):
        try:
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self.is_connected = True
            return True
        except Exception:
            self.is_connected = False
            return False

    def disconnect(self):
        try:
            if self.lidar and self.is_connected:
                self.lidar.stop()
                time.sleep(0.5)
                self.lidar.set_motor_pwm(0)
                time.sleep(0.5)
                self.lidar.disconnect()
                self.is_connected = False
        except Exception:
            pass

    def start_scanning(self, mode='ultra_stable'):
        try:
            if not self.is_connected:
                return False
            self.lidar.stop()
            time.sleep(1.0)
            self.lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(3.0)
            if mode == 'ultra_stable':
                try:
                    self.scan_generator = self.lidar.start_scan_express(self.stability_mode)
                    self.scan_iterator = self.scan_generator()
                    self.current_mode = 'stability'
                    return True
                except Exception:
                    pass
                try:
                    self.scan_generator = self.lidar.force_scan()
                    self.scan_iterator = self.scan_generator()
                    self.current_mode = 'force'
                    return True
                except Exception:
                    return False
            return True
        except Exception:
            return False

    def get_scan_data_generator(self, shutdown_flag):
        scan_buffer = []
        last_angle = None
        consecutive_failures = 0
        max_failures = 50
        while not shutdown_flag[0] and consecutive_failures < max_failures:
            try:
                measurement = next(self.scan_iterator)
                if measurement:
                    consecutive_failures = 0
                    try:
                        quality = getattr(measurement, 'quality', 0)
                        angle = getattr(measurement, 'angle', 0)
                        distance = getattr(measurement, 'distance', 0)
                        if quality > 8 and 200 < distance < 6000:
                            scan_buffer.append((quality, angle, distance))
                            if (last_angle is not None and angle < last_angle and len(scan_buffer) > 100):
                                yield scan_buffer
                                scan_buffer = []
                            last_angle = angle
                    except Exception:
                        continue
                else:
                    consecutive_failures += 1
                    time.sleep(0.1)
                    continue
            except StopIteration:
                consecutive_failures += 1
                scan_buffer.clear()
                last_angle = None
                continue
            except Exception:
                consecutive_failures += 1
                time.sleep(0.15)

class UltraStableLidarSystem:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        self.scan_queue = queue.Queue(maxsize=2)
        self.latest_obstacles = []
        self.data_lock = threading.Lock()
        self.obstacle_confidence = {}
        self.scan_cycle_count = 0
        self.angle_resolution = 1.0
        self.distance_resolution = 50
        self.confidence_threshold = 0.15
        self.confidence_increment = 0.25
        self.confidence_decay = 0.05
        self.max_confidence = 1.0
        self.scan_rate = 0
        self.last_scan_time = time.time()
        self.scan_count = 0
        self.running = False
        self.shutdown_flag = [False]
        self.threads = []

    def data_acquisition_thread(self):
        try:
            self.lidar = UltraStablePyRPLidarA3(self.port, self.baudrate, timeout=2.0)
            if not self.lidar.connect():
                return
            if not self.lidar.start_scanning('ultra_stable'):
                return
            try:
                for scan_data in self.lidar.get_scan_data_generator(self.shutdown_flag):
                    if not self.running or self.shutdown_flag[0]:
                        break
                    try:
                        if scan_data and len(scan_data) > 0:
                            self.scan_count += 1
                            current_time = time.time()
                            if current_time - self.last_scan_time >= 5.0:
                                self.scan_rate = self.scan_count / (current_time - self.last_scan_time)
                                self.scan_count = 0
                                self.last_scan_time = current_time
                            self.update_obstacle_confidence_stable(scan_data)
                            stable_obstacles = self.get_confident_obstacles()
                            if len(stable_obstacles) > 0:
                                with self.data_lock:
                                    self.latest_obstacles = stable_obstacles
                                try:
                                    self.scan_queue.put_nowait(stable_obstacles)
                                except queue.Full:
                                    try:
                                        self.scan_queue.get_nowait()
                                        self.scan_queue.put_nowait(stable_obstacles)
                                    except queue.Empty:
                                        pass
                    except Exception:
                        continue
            except Exception:
                pass
        except Exception:
            pass
        finally:
            if self.lidar:
                self.lidar.disconnect()

    def update_obstacle_confidence_stable(self, scan_data):
        seen_obstacles = set()
        for quality, angle, distance in scan_data:
            if quality > 8 and 200 < distance < 6000:
                qa = round(angle / self.angle_resolution) * self.angle_resolution
                qd = round(distance / self.distance_resolution) * self.distance_resolution
                key = (qa, qd)
                seen_obstacles.add(key)
                cur = self.obstacle_confidence.get(key, 0)
                self.obstacle_confidence[key] = min(self.max_confidence, cur + self.confidence_increment)
        to_remove = []
        for key, conf in list(self.obstacle_confidence.items()):
            if key not in seen_obstacles:
                new_conf = conf - self.confidence_decay
                if new_conf <= 0:
                    to_remove.append(key)
                else:
                    self.obstacle_confidence[key] = new_conf
        for k in to_remove:
            del self.obstacle_confidence[k]

    def get_confident_obstacles(self):
        return [(a, d) for (a, d), c in self.obstacle_confidence.items() if c >= self.confidence_threshold]

    def get_display_obstacles(self):
        with self.data_lock:
            return self.latest_obstacles.copy() if self.latest_obstacles else []

    def start(self):
        self.running = True
        self.shutdown_flag[0] = False
        t = threading.Thread(target=self.data_acquisition_thread, daemon=True)
        t.start()
        self.threads.append(t)
        return True

    def stop(self):
        self.running = False
        self.shutdown_flag[0] = True
        for t in self.threads:
            t.join(timeout=2.0)
        if self.lidar:
            self.lidar.disconnect()

# ======================================================================================
# Detection system (UNCHANGED from working version)
# ======================================================================================

class OptimizedDetectionSystem:
    def __init__(self):
        self.device = None
        self.pipeline = None
        self.detection_queue = None
        self.has_detection = False
        self.camera_initialized = False
        self.camera_resolution_width = 300
        self.camera_resolution_height = 300
        self.camera_hfov_degrees = 114
        self.target_fps = 25
        self.confidence_threshold = 0.4
        self.detection_skip_frames = 1
        self.frame_counter = 0
        self.z_depth_smoother = deque(maxlen=5)
        self.confidence_weights = deque(maxlen=5)
        self.smoothed_z_depth = 0
        self.depth_trust_threshold = 0.6

    def create_pipeline(self):
        if not dai:
            return None
        try:
            pipeline = dai.Pipeline()
            local_blob_path = "./mobilenet-ssd_openvino_2021.4_5shave.blob"
            if not os.path.exists(local_blob_path):
                print(f"⚠️ Blob file not found: {local_blob_path}")
                return None
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            mono_left.setFps(self.target_fps)
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            mono_right.setFps(self.target_fps)
            manip_nn = pipeline.create(dai.node.ImageManip)
            manip_nn.initialConfig.setResize(300, 300)
            manip_nn.initialConfig.setKeepAspectRatio(False)
            manip_nn.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
            mono_right.out.link(manip_nn.inputImage)
            depth = pipeline.create(dai.node.StereoDepth)
            depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
            depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            depth.initialConfig.setConfidenceThreshold(180)
            depth.setLeftRightCheck(True)
            depth.setSubpixel(False)
            depth.setDepthAlign(dai.CameraBoardSocket.CAM_C)
            mono_left.out.link(depth.left)
            mono_right.out.link(depth.right)
            detection_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
            detection_nn.setConfidenceThreshold(0.4)
            detection_nn.setBlobPath(local_blob_path)
            detection_nn.setBoundingBoxScaleFactor(0.5)
            detection_nn.setDepthLowerThreshold(100)
            detection_nn.setDepthUpperThreshold(8000)
            manip_nn.out.link(detection_nn.input)
            depth.depth.link(detection_nn.inputDepth)
            detection_out = pipeline.create(dai.node.XLinkOut)
            detection_out.setStreamName("detections")
            detection_nn.out.link(detection_out.input)
            return pipeline
        except Exception as e:
            print(f"Pipeline creation error: {e}")
            return None

    def initialize(self):
        if not dai:
            print("⚠️ DepthAI not available - using fallback")
            return False
        try:
            devices = dai.Device.getAllAvailableDevices()
            if len(devices) == 0:
                print("⚠️ No OAK devices found")
                return False
            self.pipeline = self.create_pipeline()
            if not self.pipeline:
                return False
            self.device = dai.Device(self.pipeline)
            try:
                if hasattr(self.device, 'setLogLevel'):
                    self.device.setLogLevel(dai.LogLevel.WARN)
                self.device.setIrLaserDotProjectorIntensity(900)
            except Exception:
                pass
            try:
                self.detection_queue = self.device.getOutputQueue("detections", maxSize=4, blocking=False)
                self.has_detection = True
            except Exception:
                self.detection_queue = None
                self.has_detection = False
            time.sleep(2)
            self.camera_initialized = True
            print("✅ Optimized detection system initialized")
            return True
        except Exception as e:
            print(f"Detection system initialization error: {e}")
            return False

    def smooth_z_depth(self, raw_z_depth, confidence):
        try:
            self.z_depth_smoother.append(raw_z_depth)
            self.confidence_weights.append(confidence)
            if len(self.z_depth_smoother) < 2:
                self.smoothed_z_depth = raw_z_depth
                return raw_z_depth
            total_weight = 0
            weighted_sum = 0
            for i, (z_val, conf) in enumerate(zip(self.z_depth_smoother, self.confidence_weights)):
                recency_weight = (i + 1) / len(self.z_depth_smoother)
                confidence_weight = max(0.1, conf)
                combined_weight = recency_weight * confidence_weight
                weighted_sum += z_val * combined_weight
                total_weight += combined_weight
            smoothed = weighted_sum / total_weight if total_weight > 0 else raw_z_depth
            if confidence < self.depth_trust_threshold and len(self.z_depth_smoother) > 1:
                prev_z = self.z_depth_smoother[-2]
                max_jump = 500
                if abs(smoothed - prev_z) > max_jump:
                    blend_factor = confidence / self.depth_trust_threshold
                    smoothed = prev_z + (smoothed - prev_z) * blend_factor
            self.smoothed_z_depth = smoothed
            return smoothed
        except Exception:
            return raw_z_depth

    def get_detection(self):
        if not self.has_detection or not self.detection_queue or not self.camera_initialized:
            return None
        try:
            self.frame_counter += 1
            if self.frame_counter % self.detection_skip_frames != 0:
                return None
            detections = self.detection_queue.tryGet()
            if not detections:
                return None
            person_detections = [det for det in detections.detections if det.label == 15]
            if not person_detections:
                return None
            closest_person = min(person_detections, key=lambda p: p.spatialCoordinates.z)
            x_camera = closest_person.spatialCoordinates.x
            y_camera = closest_person.spatialCoordinates.y
            raw_z_depth = closest_person.spatialCoordinates.z
            confidence = closest_person.confidence
            if raw_z_depth <= 0 or raw_z_depth > 15000:
                return None
            smoothed_z_depth = self.smooth_z_depth(raw_z_depth, confidence)
            bbox_xmin = closest_person.xmin
            bbox_xmax = closest_person.xmax
            bbox_ymin = closest_person.ymin
            bbox_ymax = closest_person.ymax
            x_mid_norm = (bbox_xmin + bbox_xmax) / 2.0
            y_mid_norm = (bbox_ymin + bbox_ymax) / 2.0
            return {
                'x_camera': x_camera,
                'y_camera': y_camera,
                'z_camera': smoothed_z_depth,
                'raw_z_depth': raw_z_depth,
                'confidence': confidence,
                'bbox_center': {
                    'x_normalized': x_mid_norm,
                    'y_normalized': y_mid_norm,
                },
                'bounding_box': {
                    'xmin': bbox_xmin,
                    'xmax': bbox_xmax,
                    'ymin': bbox_ymin,
                    'ymax': bbox_ymax
                }
            }
        except Exception:
            return None

    def shutdown(self):
        try:
            if self.device:
                self.device.close()
                self.device = None
        except Exception:
            pass

# ======================================================================================
# MAIN BEHAVIOR - Working version + coordinate verification
# ======================================================================================

class CoordinateVerificationLidarTest(MaxineBehavior):
    def __init__(self):
        super().__init__("Coordinate Verification LiDAR Test")
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
        self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        self.detection_system = None
        self.lidar_system = None
        self.screen = None
        self.initialized = False
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.update_counter = 0
        self.display_update_rate = 3
        
        # CSV - OVERWRITE each run + coordinate verification
        self.csv_log_filename = "COORDINATE_VERIFICATION.csv"
        self.csv_initialized = False
        self.mode_start_time = time.time()
        self.x_midpoints_pixels = deque(maxlen=1000)
        self.x_midpoints_normalized = deque(maxlen=1000)
        self.variance_window = deque(maxlen=100)
        self.current_variance_pixels = 0.0
        self.current_std_dev_pixels = 0.0
        self.current_mean_pixels = 0.0
        self.detection_count = 0
        self.consistent_detection_count = 0
        self.large_jumps_count = 0
        self.jump_threshold_pixels = 50
        self.last_x_midpoint = None
        self.short_term_variance = deque(maxlen=75)
        self.medium_term_variance = deque(maxlen=250)
        self.long_term_variance = deque(maxlen=1500)
        self.stability_zones = {'stable': 0, 'moderate': 0, 'unstable': 0, 'very_unstable': 0}
        
        # Head tracking - UNCHANGED from working version
        self.head_tracker = None
        self.head_tracking_enabled = True
        self._head_filter = _HeadCommandFilter()  # Uses faster config now
        self.angle_history: list[float] = []
        self.max_angle_history = 3
        self.last_sent_angle: float | None = None
        self.tracking_update_interval = 1   # update every tick (unchanged)
        self.dead_zone_degrees = 2          # unchanged
        self.update_counter_tracking = 0
        self.last_person_detected = 0.0
        self.person_lost_timeout = 2.0
        # lead estimation state (unchanged)
        self._last_raw_yaw: float | None = None
        self._last_raw_time: float | None = None
        # detection cache so control runs every tick (unchanged)
        self._cached_person_data = None
        self._cache_t = 0.0
        
        # COORDINATE VERIFICATION (NEW - added to working version)
        self.coordinate_matches = deque(maxlen=100)
        self.alignment_tolerance_degrees = 5.0  # ±5 degrees tolerance
        self.detection_radius_tolerance = 500   # ±500mm tolerance
        
        if not pygame.get_init():
            pygame.init()
        pygame.font.init()

    def setup(self, **kwargs):
        return True

    def initialise(self):
        print("🎯 Coordinate Verification (WORKING BASE) initializing...")
        if not self.initialized:
            self.initialize_components()
        self.stop_robot()
        self.mode_start_time = time.time()
        self.csv_initialized = False
        self.detection_count = 0
        self.consistent_detection_count = 0
        self.large_jumps_count = 0
        self.last_x_midpoint = None
        self.x_midpoints_pixels.clear()
        self.x_midpoints_normalized.clear()
        self.variance_window.clear()
        self.short_term_variance.clear()
        self.medium_term_variance.clear()
        self.long_term_variance.clear()
        self.coordinate_matches.clear()
        for k in self.stability_zones:
            self.stability_zones[k] = 0

    def initialize_components(self):
        if self.initialized:
            return True
        try:
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("MAXINE COORDINATE VERIFICATION")
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            self.draw_clean_interface()
            pygame.display.flip()
            print("🎯 Initializing optimized detection system...")
            self.detection_system = OptimizedDetectionSystem()
            if self.detection_system.initialize():
                print("✅ Optimized detection system ready")
            else:
                print("⚠️ Using fallback detection")
            self.start_stable_lidar()
            self.initialize_csv_log()
            if self.head_tracking_enabled:
                self.initialize_head_tracker()
            self.initialized = True
            return True
        except Exception as e:
            print(f"Initialization error: {e}")
            self.initialized = False
            return False

    def initialize_head_tracker(self):
        """EXACT COPY from working OptimizedLidarTestMode.py"""
        try:
            robot = self.get_robot()
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                print("🎯 Initializing faster head tracking with servo controller...")
                try:
                    from src.behaviors.lidarchase.HeadTracker import HeadTracker
                    self.head_tracker = HeadTracker(head_velocity_manager=None, servo_controller=robot.servo_controller)
                    self.head_tracker.start_tracking()
                    self.head_tracker.set_manual_position(0.0)
                    print("✅ Faster head tracking initialized (servo controller)")
                except ImportError:
                    print("⚠️ HeadTracker not available, trying direct servo control...")
                    robot.servo_controller.center()
                    self.head_tracker = None
            elif hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
                print("🎯 Initializing faster head tracking with velocity manager...")
                try:
                    from src.behaviors.lidarchase.HeadTracker import HeadTracker
                    self.head_tracker = HeadTracker(head_velocity_manager=robot.head_velocity_manager, servo_controller=None)
                    self.head_tracker.start_tracking()
                    self.head_tracker.set_manual_position(0.0)
                    print("✅ Faster head tracking initialized (velocity manager)")
                except ImportError:
                    print("⚠️ HeadTracker not available")
                    self.head_tracker = None
            else:
                print("ℹ️ No head control available - head tracking disabled")
                self.head_tracker = None
                self.head_tracking_enabled = False
        except Exception as e:
            print(f"⚠️ Faster head tracking initialization failed: {e}")
            print("   Continuing without head tracking...")
            self.head_tracker = None
            self.head_tracking_enabled = False

    def initialize_csv_log(self):
        """COORDINATE VERIFICATION CSV (NEW - added to working version)"""
        try:
            if os.path.exists(self.csv_log_filename):
                os.remove(self.csv_log_filename)
                print(f"🗑️ Removed previous CSV: {self.csv_log_filename}")
                
            with open(self.csv_log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'timestamp', 'mode_time_elapsed',
                    # Person detection (camera coordinates)
                    'person_detected', 'person_x_camera', 'person_z_camera', 'person_confidence',
                    # Person in robot coordinates
                    'person_robot_angle_deg', 'person_robot_distance', 'person_transform_confidence',
                    # Head state
                    'head_angle_deg', 'head_tracking_active',
                    # LiDAR data
                    'lidar_obstacles_count', 'closest_lidar_angle_deg', 'closest_lidar_distance',
                    # Coordinate verification metrics
                    'person_lidar_angle_diff_deg', 'person_lidar_distance_diff', 'coordinates_aligned',
                    'alignment_score', 'detection_quality'
                ])
            self.csv_initialized = True
            print(f"✅ Coordinate verification CSV logging to: {self.csv_log_filename}")
        except Exception as e:
            print(f"⚠️ CSV initialization failed: {e}")

    def start_stable_lidar(self):
        """EXACT COPY from working version"""
        try:
            if not self.lidar_system:
                print("🚀 Starting LiDAR system...")
                self.lidar_system = UltraStableLidarSystem()
                success = self.lidar_system.start()
                if success:
                    self.blackboard.set("LIDAR_SYSTEM", self.lidar_system)
                    print("✅ LiDAR system started")
                    time.sleep(3)
                else:
                    print("❌ Failed to start LiDAR")
                    self.lidar_system = None
        except Exception as e:
            print(f"❌ LiDAR error: {e}")
            self.lidar_system = None

    def get_person_detection(self):
        """EXACT COPY from working version"""
        if self.detection_system and self.detection_system.camera_initialized:
            detection = self.detection_system.get_detection()
            if detection:
                screen_width = self.screen.get_width() if self.screen else 1920
                screen_height = self.screen.get_height() if self.screen else 1080
                x_pixels = int(detection['bbox_center']['x_normalized'] * screen_width)
                y_pixels = int(detection['bbox_center']['y_normalized'] * screen_height)
                detection['bbox_center']['x_pixels'] = x_pixels
                detection['bbox_center']['y_pixels'] = y_pixels
                return detection
        try:
            robot = self.get_robot()
            if not hasattr(robot, 'camera_sensor') or not robot.camera_sensor:
                return None
            reading = robot.camera_sensor.get_reading()
            if not reading or not hasattr(reading, 'get_people_locations'):
                return None
            people = reading.get_people_locations()
            if not people:
                return None
            closest_person = people[0]
            if closest_person.confidence < 0.4:
                return None
            x_center_normalized = (closest_person.xmax + closest_person.xmin) / 2.0
            y_center_normalized = (closest_person.ymax + closest_person.ymin) / 2.0
            screen_width = self.screen.get_width() if self.screen else 1920
            screen_height = self.screen.get_height() if self.screen else 1080
            x_center_pixels = int(x_center_normalized * screen_width)
            y_center_pixels = int(y_center_normalized * screen_height)
            return {
                'x_camera': closest_person.spatialCoordinates.x,
                'z_camera': closest_person.spatialCoordinates.z,
                'confidence': closest_person.confidence,
                'bbox_center': {
                    'x_normalized': x_center_normalized,
                    'y_normalized': y_center_normalized,
                    'x_pixels': x_center_pixels,
                    'y_pixels': y_center_pixels
                },
                'bounding_box': {
                    'xmin': closest_person.xmin,
                    'xmax': closest_person.xmax,
                    'ymin': closest_person.ymin,
                    'ymax': closest_person.ymax
                }
            }
        except Exception:
            return None

    # Drawing methods (EXACT COPY from working version)
    def draw_clean_interface(self):
        self.screen.fill((0, 0, 0))
        self.draw_radar_grid()
        self.draw_robot()

    def draw_radar_grid(self):
        for distance in [1000, 2000, 3000, 4000, 5000, 6000]:
            radius = distance * self.scale // 1000
            if radius < min(self.center_x, self.center_y) - 50:
                line_width = 3 if distance == 6000 else 2
                color = (0, 150, 0) if distance < 6000 else (255, 255, 0)
                pygame.draw.circle(self.screen, color, (self.center_x, self.center_y), radius, line_width)
        for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
            display_angle_rad = math.radians(90 - angle)
            line_length = min(self.center_x, self.center_y) - 80
            end_x = self.center_x + int(line_length * math.cos(display_angle_rad))
            end_y = self.center_y - int(line_length * math.sin(display_angle_rad))
            line_width = 3 if angle % 90 == 0 else 1
            pygame.draw.line(self.screen, (0, 150, 0), (self.center_x, self.center_y), (end_x, end_y), line_width)

    def draw_robot(self):
        pygame.draw.circle(self.screen, (0, 255, 0), (self.center_x, self.center_y), 15, 3)
        arrow_end_x = self.center_x
        arrow_end_y = self.center_y - 30
        pygame.draw.line(self.screen, (0, 255, 0), (self.center_x, self.center_y), (arrow_end_x, arrow_end_y), 5)

    def draw_lidar_data(self, obstacles):
        if not obstacles:
            return 0
        obstacle_count = 0
        for angle, distance in obstacles:
            corrected_angle_rad = math.radians(90 - angle)
            x = self.center_x + math.cos(corrected_angle_rad) * (distance * self.scale // 1000)
            y = self.center_y - math.sin(corrected_angle_rad) * (distance * self.scale // 1000)
            if (0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height()):
                pygame.draw.circle(self.screen, (255, 255, 255), (int(x), int(y)), 2)
                obstacle_count += 1
        return obstacle_count

    def draw_person_detection(self, person_data=None):
        """MODIFIED to show coordinate alignment"""
        if person_data is None:
            person_data = self.get_person_detection()
        if not person_data:
            return None
        try:
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            bbox_center = person_data['bbox_center']
            if z_camera <= 0:
                return None
            
            # Log detection data for coordinate verification
            self.log_detection_consistency_to_csv(person_data)
            
            person_angle_rad = math.atan2(x_camera, z_camera)
            person_angle_deg = math.degrees(person_angle_rad)
            display_angle_deg = 360 - person_angle_deg
            while display_angle_deg < 0:
                display_angle_deg += 360
            while display_angle_deg >= 360:
                display_angle_deg -= 360
            
            distance_m = z_camera / 1000.0
            display_angle_rad = math.radians(90 - display_angle_deg)
            x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
            y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
            
            stability_class = self.classify_stability(self.current_std_dev_pixels)
            color = (0, 255, 0) if stability_class == 'stable' else \
                    (255, 255, 0) if stability_class == 'moderate' else \
                    (255, 165, 0) if stability_class == 'unstable' else (255, 0, 0)
            
            if 0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height():
                pygame.draw.circle(self.screen, color, (x, y), 8)
                pygame.draw.circle(self.screen, (255, 255, 255), (x, y), 9, 2)
            
            self.draw_person_x_coordinate_info(bbox_center)
            return person_data
        except Exception:
            return None

    def draw_person_x_coordinate_info(self, bbox_center):
        """EXACT COPY from working version"""
        try:
            screen_width = self.screen.get_width()
            x_norm = bbox_center['x_normalized']
            x_pixels = bbox_center['x_pixels']
            large_font = pygame.font.Font(None, 72)
            medium_font = pygame.font.Font(None, 48)
            main_text = f"Person X-Center: {x_pixels}px"
            detail_text = f"Variance: ±{self.current_std_dev_pixels:.1f}px | Target: <37px | Head: {self.get_head_angle_degrees():.1f}°"
            if self.current_std_dev_pixels <= 37:
                main_color = (0, 255, 0)
            elif self.current_std_dev_pixels <= 50:
                main_color = (255, 255, 0)
            else:
                main_color = (255, 0, 0)
            main_surface = large_font.render(main_text, True, main_color)
            detail_surface = medium_font.render(detail_text, True, (255, 255, 255))
            main_x = (screen_width - main_surface.get_width()) // 2
            main_y = 50
            detail_x = (screen_width - detail_surface.get_width()) // 2
            detail_y = main_y + main_surface.get_height() + 10
            self.screen.blit(main_surface, (main_x, main_y))
            self.screen.blit(detail_surface, (detail_x, detail_y))
        except Exception:
            pass

    # Head tracking (EXACT COPY from working version)
    def _compute_pixel_angle(self, x_pixels: int) -> float:
        screen_center_x = self.screen.get_width() // 2 if self.screen else 960
        pixel_offset = x_pixels - screen_center_x
        pixel_offset_normalized = pixel_offset / max(1, screen_center_x)
        hfov_deg = getattr(self.detection_system, 'camera_hfov_degrees', 114)
        camera_hfov_rad = math.radians(hfov_deg)
        return -pixel_offset_normalized * (camera_hfov_rad / 2.0)

    def _range_based_pitch(self, z_camera_mm: float) -> float:
        return _clamp(math.radians(10) - math.atan2(0.2, max(z_camera_mm / 1000.0, 0.2)),
                      math.radians(-30), math.radians(30))

    def _lead_yaw(self, raw_yaw: float) -> float:
        now = time.time()
        if self._last_raw_yaw is None or self._last_raw_time is None:
            self._last_raw_yaw = raw_yaw
            self._last_raw_time = now
            return raw_yaw
        dt = max(1e-3, now - self._last_raw_time)
        vel = (raw_yaw - self._last_raw_yaw) / dt
        self._last_raw_yaw = raw_yaw
        self._last_raw_time = now
        lead = raw_yaw + vel * self._head_filter.cfg.lead_time_s
        return _wrap_pi(lead)

    def update_head_tracking(self, person_data):
        """EXACT COPY from working version"""
        if not self.head_tracking_enabled or not self.head_tracker or not person_data:
            return
        try:
            bbox_center = person_data['bbox_center']
            x_pixels = bbox_center['x_pixels']
            z_camera = person_data.get('z_camera', 0)
            if z_camera <= 0:
                return
            self.update_counter_tracking += 1
            if self.update_counter_tracking % self.tracking_update_interval != 0:
                return
            raw_yaw = self._compute_pixel_angle(x_pixels)
            dead_zone_rad = math.radians(self.dead_zone_degrees)
            if abs(raw_yaw) <= dead_zone_rad and (self.last_sent_angle is None or abs(self.last_sent_angle) <= dead_zone_rad):
                self.last_person_detected = time.time()
                return
            target_yaw = self._lead_yaw(raw_yaw)
            desired_pitch = self._range_based_pitch(z_camera)
            yaw_cmd, pitch_cmd = self._head_filter.update(target_yaw, desired_pitch, person_locked=True)
            self.head_tracker.set_person_tracking(yaw_cmd)
            self.last_sent_angle = yaw_cmd
            self.last_person_detected = time.time()
        except Exception:
            pass

    def get_head_angle_degrees(self):
        """EXACT COPY from working version"""
        return math.degrees(self.last_sent_angle) if self.last_sent_angle is not None else 0.0

    def draw_info(self, obstacle_count):
        """MODIFIED to show coordinate verification"""
        try:
            lidar_status = "ACTIVE" if self.lidar_system else "INACTIVE"
            detection_status = "OPTIMIZED" if (self.detection_system and self.detection_system.camera_initialized) else "FALLBACK"
            consistency_rate = (self.consistent_detection_count / max(1, self.detection_count)) * 100
            if self.head_tracker and self.head_tracking_enabled:
                head_angle = self.get_head_angle_degrees()
                head_status = f"HEAD: {head_angle:.1f}° (TRACKING)"
            elif self.head_tracking_enabled:
                head_status = "HEAD: Initializing tracking..."
            else:
                head_status = "HEAD: Disabled"
            info_lines = [
                f"COORDINATE VERIFICATION - Detection: {detection_status} | LiDAR: {lidar_status}",
                f"Obstacles: {obstacle_count} | Consistency: {consistency_rate:.1f}% | {head_status}",
                f"Variance: ±{self.current_std_dev_pixels:.1f}px (Target <37px) | Detections: {self.detection_count}",
                f"CSV: {self.csv_log_filename} | Press ESC to exit to IDLE mode"
            ]
            y_offset = self.screen.get_height() - 120
            font = pygame.font.Font(None, 36)
            for i, line in enumerate(info_lines):
                color = (0, 255, 255) if i == 0 else (255, 255, 255)
                if "Variance" in line:
                    if self.current_std_dev_pixels <= 10:
                        color = (0, 255, 0)
                    elif self.current_std_dev_pixels <= 37:
                        color = (255, 255, 0)
                    else:
                        color = (255, 0, 0)
                text_surface = font.render(line, True, color)
                self.screen.blit(text_surface, (20, y_offset + i * 30))
        except Exception:
            pass

    # Stats/CSV (MODIFIED for coordinate verification)
    def calculate_x_midpoint_variance(self):
        """EXACT COPY from working version"""
        try:
            if len(self.variance_window) < 2:
                return 0.0, 0.0, 0.0
            window_data = list(self.variance_window)
            mean_val = statistics.mean(window_data)
            variance_val = statistics.variance(window_data) if len(window_data) > 1 else 0.0
            std_dev_val = math.sqrt(variance_val)
            return variance_val, std_dev_val, mean_val
        except Exception:
            return 0.0, 0.0, 0.0

    def calculate_multi_term_variance(self, x_pixel):
        """EXACT COPY from working version"""
        self.short_term_variance.append(x_pixel)
        self.medium_term_variance.append(x_pixel)
        self.long_term_variance.append(x_pixel)
        try:
            short_var = statistics.variance(list(self.short_term_variance)) if len(self.short_term_variance) > 1 else 0.0
            medium_var = statistics.variance(list(self.medium_term_variance)) if len(self.medium_term_variance) > 1 else 0.0
            long_var = statistics.variance(list(self.long_term_variance)) if len(self.long_term_variance) > 1 else 0.0
            return short_var, medium_var, long_var
        except Exception:
            return 0.0, 0.0, 0.0

    def classify_stability(self, std_dev_pixels):
        """EXACT COPY from working version"""
        if std_dev_pixels < 10:
            return 'stable'
        elif std_dev_pixels < 25:
            return 'moderate'
        elif std_dev_pixels < 50:
            return 'unstable'
        else:
            return 'very_unstable'

    def log_detection_consistency_to_csv(self, person_data):
        """MODIFIED for coordinate verification"""
        try:
            if not self.csv_initialized:
                self.initialize_csv_log()
            
            # Basic person detection data
            bbox_center = person_data['bbox_center']
            x_midpoint_pixels = bbox_center['x_pixels']
            x_midpoint_normalized = bbox_center['x_normalized']
            self.x_midpoints_pixels.append(x_midpoint_pixels)
            self.x_midpoints_normalized.append(x_midpoint_normalized)
            self.variance_window.append(x_midpoint_pixels)
            
            # Standard detection consistency tracking
            x_jump = 0
            is_large_jump = False
            if self.last_x_midpoint is not None:
                x_jump = abs(x_midpoint_pixels - self.last_x_midpoint)
                is_large_jump = x_jump > self.jump_threshold_pixels
                if is_large_jump:
                    self.large_jumps_count += 1
            self.last_x_midpoint = x_midpoint_pixels
            self.detection_count += 1
            if not is_large_jump:
                self.consistent_detection_count += 1
            
            self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels = self.calculate_x_midpoint_variance()
            short_var, medium_var, long_var = self.calculate_multi_term_variance(x_midpoint_pixels)
            self.stability_zones[self.classify_stability(self.current_std_dev_pixels)] += 1
            
            # COORDINATE VERIFICATION ANALYSIS (NEW)
            current_time = time.time()
            mode_elapsed = current_time - self.mode_start_time
            
            # Get current head angle
            head_angle_deg = self.get_head_angle_degrees()
            head_angle_rad = math.radians(head_angle_deg)
            
            # Transform person from camera to robot coordinates
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            camera_angle = math.atan2(x_camera, z_camera) if z_camera > 0 else 0.0
            robot_angle = camera_angle + head_angle_rad
            # Normalize angle
            while robot_angle > math.pi:
                robot_angle -= 2 * math.pi
            while robot_angle < -math.pi:
                robot_angle += 2 * math.pi
            robot_angle_deg = math.degrees(robot_angle)
            robot_distance = max(100, z_camera - 130)  # Adjust for camera offset
            transform_confidence = 1.0
            
            # Get LiDAR obstacles
            lidar_obstacles = []
            if self.lidar_system:
                lidar_obstacles = self.lidar_system.get_display_obstacles()
            
            # Find closest LiDAR obstacle to person
            closest_lidar_angle = 0
            closest_lidar_distance = 0
            angle_diff = 999
            distance_diff = 9999
            coordinates_aligned = False
            alignment_score = 0.0
            detection_quality = 0
            
            if lidar_obstacles:
                # Find best match
                best_match = None
                best_score = 0
                for lidar_angle, lidar_distance in lidar_obstacles:
                    # Convert LiDAR angle to same range as robot angle
                    lidar_norm = lidar_angle if lidar_angle <= 180 else lidar_angle - 360
                    ang_diff = abs(robot_angle_deg - lidar_norm)
                    if ang_diff > 180:
                        ang_diff = 360 - ang_diff
                    dist_diff = abs(robot_distance - lidar_distance)
                    
                    if ang_diff <= 20:  # Reasonable match range
                        match_score = 1.0 / (1.0 + ang_diff + dist_diff/1000.0)
                        if match_score > best_score:
                            best_score = match_score
                            best_match = (lidar_angle, lidar_distance, ang_diff, dist_diff)
                
                if best_match:
                    closest_lidar_angle, closest_lidar_distance, angle_diff, distance_diff = best_match
                    coordinates_aligned = (angle_diff <= self.alignment_tolerance_degrees and 
                                         distance_diff <= self.detection_radius_tolerance)
                    angle_score = max(0, 1.0 - angle_diff / 45.0)
                    distance_score = max(0, 1.0 - distance_diff / 2000.0)
                    alignment_score = (angle_score + distance_score) / 2.0
                    detection_quality = len([obs for obs in lidar_obstacles if abs(obs[0] - closest_lidar_angle) <= 10])
            
            # Log to CSV
            with open(self.csv_log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    current_time, mode_elapsed,
                    True, x_camera, z_camera, person_data.get('confidence', 0.0),
                    robot_angle_deg, robot_distance, transform_confidence,
                    head_angle_deg, self.head_tracking_enabled,
                    len(lidar_obstacles), closest_lidar_angle, closest_lidar_distance,
                    angle_diff, distance_diff, coordinates_aligned,
                    alignment_score, detection_quality
                ])
                
        except Exception as e:
            print(f"CSV logging error: {e}")

    # Behavior lifecycle (EXACT COPY from working version)
    def stop_robot(self):
        try:
            robot = self.get_robot()
            velocity_manager = None
            if hasattr(robot, 'direct_velocity_manager') and robot.direct_velocity_manager:
                velocity_manager = robot.direct_velocity_manager
            elif hasattr(robot, 'velocity_manager') and robot.velocity_manager:
                velocity_manager = robot.velocity_manager
            if velocity_manager:
                stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                velocity_manager.perform_action(stop_config)
        except Exception:
            pass

    def update(self) -> Status:
        """EXACT COPY from working version + coordinate verification"""
        try:
            if not self.initialized:
                if not self.initialize_components():
                    return Status.FAILURE
            self.update_counter += 1
            pygame.event.pump()

            # Always poll detection + update head every tick (UNCHANGED from working version)
            person_data = self.get_person_detection()
            if person_data:
                self._cached_person_data = person_data
                self._cache_t = time.time()
                if self.head_tracking_enabled:
                    self.update_head_tracking(person_data)
            else:
                if self._cached_person_data and (time.time() - self._cache_t) <= 0.25 and self.head_tracking_enabled:
                    self.update_head_tracking(self._cached_person_data)
                elif self.head_tracker and time.time() - self.last_person_detected > self.person_lost_timeout:
                    try:
                        self.angle_history.clear()
                        self.last_sent_angle = None
                        self.head_tracker.set_manual_position(0.0)
                    except Exception:
                        pass

            # Render less often to save CPU (unchanged from working version)
            if self.update_counter % self.display_update_rate == 0:
                try:
                    if self.screen:
                        self.screen.fill((0, 0, 0))
                        self.draw_radar_grid()
                        self.draw_robot()
                        obstacle_count = 0
                        if self.lidar_system:
                            obstacles = self.lidar_system.get_display_obstacles()
                            if obstacles:
                                obstacle_count = self.draw_lidar_data(obstacles)
                        self.draw_person_detection(self._cached_person_data)
                        self.draw_info(obstacle_count)
                        pygame.display.flip()
                except Exception as e:
                    print(f"Display error: {e}")
            return Status.RUNNING
        except Exception as e:
            print(f"Update error: {e}")
            return Status.RUNNING

    def terminate(self, new_status: Status):
        """EXACT COPY from working version"""
        print("🔥 Coordinate Verification terminating properly via py_trees...")
        try:
            self.stop_robot()
            if self.head_tracker:
                print("🛑 Stopping head tracking...")
                try:
                    self.head_tracker.set_manual_position(0.0)
                    time.sleep(0.5)
                    self.head_tracker.stop_tracking()
                except Exception as e:
                    print(f"   Head tracking cleanup warning: {e}")
                self.head_tracker = None
            elif self.head_tracking_enabled:
                try:
                    robot = self.get_robot()
                    if hasattr(robot, 'servo_controller') and robot.servo_controller:
                        robot.servo_controller.center()
                    elif hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
                        robot.head_velocity_manager.center_head()
                except Exception:
                    pass
            if self.lidar_system:
                print("🛑 Stopping LiDAR system...")
                self.lidar_system.stop()
                self.lidar_system = None
            if self.detection_system:
                print("🛑 Stopping detection system...")
                self.detection_system.shutdown()
                self.detection_system = None
            try:
                if self.blackboard.exists("LIDAR_SYSTEM"):
                    self.blackboard.unset("LIDAR_SYSTEM")
            except Exception:
                pass
            if self.detection_count > 0:
                overall_variance = statistics.variance(list(self.x_midpoints_pixels)) if len(self.x_midpoints_pixels) > 1 else 0
                overall_std_dev = math.sqrt(overall_variance)
                consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
                print(f"\n📊 COORDINATE VERIFICATION FINAL SUMMARY:")
                print(f"   Total Detections: {self.detection_count}")
                print(f"   Consistency Rate: {consistency_rate:.1f}%")
                print(f"   Overall Std Dev: ±{overall_std_dev:.2f} pixels")
                print(f"   Target Met: {'✅ YES' if overall_std_dev < 37 else '❌ NO'} (target <37px)")
                print(f"   Head Tracking: {'Working' if self.head_tracking_enabled else 'Disabled'}")
                print(f"   CSV Data: {self.csv_log_filename}")
            if pygame.get_init():
                pygame.event.clear()
            self.initialized = False
            print("✅ Coordinate Verification terminated successfully")
        except Exception as e:
            print(f"⚠️ Termination error: {e}")
        super().terminate(new_status)

# Export for use
__all__ = ['CoordinateVerificationLidarTest']