#!/usr/bin/env python3
"""
Enhanced LiDAR Test Mode for Maxine the Robot
Enhanced version with improved facial animation restoration for proper IDLE mode transition
Uses ultra-stable LiDAR system with comprehensive cleanup procedures
Now includes Head Align functionality to use recorded center position
"""
import pygame
import math
import time
import threading
import queue
from typing import List, Optional, Tuple
from pyrplidar import PyRPlidar
import py_trees
from py_trees.common import Status

from ...behaviors.MaxineBehavior import MaxineBehavior
from ...types.RobotModes import RobotMode
from ...types.MovementDirection import MovementDirection
from ...types.KeyboardKey import KeyboardKey
from ...action_managers.VelocityManager import VelocityConfig


class UltraStablePyRPLidarA3:
    """Ultra-stable LiDAR class optimized for test operations"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, timeout=2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None
        
        # Ultra-Stable Parameters
        self.motor_pwm = 600
        self.stability_mode = 4
        self.current_mode = None

    def connect(self):
        """Connect to the LiDAR device"""
        try:
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self.is_connected = True
            return True
        except Exception:
            self.is_connected = False
            return False

    def disconnect(self):
        """Graceful disconnect from the LiDAR device"""
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
        """Start scanning using ultra-stable settings"""
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
                    pass
            
            elif mode == 'force_only':
                self.scan_generator = self.lidar.force_scan()
                self.scan_iterator = self.scan_generator()
                self.current_mode = 'force'
                return True
            
            return False
            
        except Exception:
            return False

    def get_scan_data_generator(self, shutdown_flag):
        """Ultra-stable generator for test operations"""
        if not self.is_connected or not self.lidar or not self.scan_iterator:
            return
            
        consecutive_failures = 0
        max_failures = 100
        scan_buffer = []
        last_angle = None
        min_scan_points = 150
        restart_count = 0
        max_restarts = 2
        
        while consecutive_failures < max_failures:
            if shutdown_flag[0]:
                break
                
            try:
                measurement = next(self.scan_iterator)
                
                if measurement:
                    consecutive_failures = 0
                    
                    try:
                        quality = getattr(measurement, 'quality', 0)
                        angle = getattr(measurement, 'angle', 0)
                        distance = getattr(measurement, 'distance', 0)
                        
                        if quality > 5 and distance > 100 and distance < 6000:
                            scan_buffer.append((quality, angle, distance))
                            
                            if (last_angle is not None and 
                                angle < last_angle and 
                                len(scan_buffer) > min_scan_points):
                                yield scan_buffer.copy()
                                scan_buffer.clear()
                            
                            last_angle = angle
                        
                    except Exception:
                        continue
                else:
                    consecutive_failures += 1
                    time.sleep(0.02)
                    
            except StopIteration:
                restart_count += 1
                if restart_count >= max_restarts:
                    break
                
                time.sleep(2)
                
                if self.start_scanning('ultra_stable'):
                    consecutive_failures = 0
                    scan_buffer.clear()
                    last_angle = None
                    continue
                elif self.start_scanning('force_only'):
                    consecutive_failures = 0
                    scan_buffer.clear()
                    last_angle = None
                    continue
                else:
                    break
                    
            except Exception:
                consecutive_failures += 1
                time.sleep(0.15)


class UltraStableLidarTestSystem:
    """Ultra-stable LiDAR system for test operations"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        
        # Ultra-stable data structures
        self.scan_queue = queue.Queue(maxsize=2)
        self.latest_obstacles = []
        self.data_lock = threading.Lock()
        
        # Test-optimized obstacle mapping
        self.obstacle_confidence = {}
        self.scan_cycle_count = 0
        self.angle_resolution = 1.0
        self.distance_resolution = 50
        
        # Test-optimized confidence parameters
        self.confidence_threshold = 0.3        
        self.confidence_increment = 0.4        
        self.confidence_decay = 0.25           
        self.max_confidence = 1.0
        
        # Performance monitoring
        self.scan_rate = 0
        self.last_scan_time = time.time()
        self.scan_count = 0
        self.processing_time = 0
        
        # Control flags
        self.running = False
        self.shutdown_flag = [False]
        self.threads = []
        
    def data_acquisition_thread(self):
        """Ultra-stable data acquisition thread for testing"""
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
                    
                    start_time = time.time()
                    
                    try:
                        if scan_data and len(scan_data) > 0:
                            self.scan_count += 1
                            current_time = time.time()
                            if current_time - self.last_scan_time >= 5.0:
                                self.scan_rate = self.scan_count / (current_time - self.last_scan_time)
                                self.scan_count = 0
                                self.last_scan_time = current_time
                                self.processing_time = 0
                            
                            self.update_obstacle_confidence_test(scan_data)
                            stable_obstacles = self.get_confident_obstacles()
                            
                            if len(stable_obstacles) > 0:
                                with self.data_lock:
                                    self.latest_obstacles = stable_obstacles
                                
                                try:
                                    while not self.scan_queue.empty():
                                        self.scan_queue.get_nowait()
                                    self.scan_queue.put_nowait((stable_obstacles, self.lidar.current_mode, self.scan_rate))
                                except (queue.Full, queue.Empty):
                                    pass
                            
                            self.processing_time += time.time() - start_time
                        
                    except Exception:
                        if not self.shutdown_flag[0]:
                            continue
                        
            except Exception:
                pass
                
        except Exception:
            pass
        finally:
            if self.lidar:
                self.lidar.disconnect()
    
    def update_obstacle_confidence_test(self, scan_data):
        """Test-optimized confidence update with faster decay"""
        self.scan_cycle_count += 1
        seen_obstacles = set()
        
        for quality, angle, distance in scan_data:
            if distance > 200 and quality > 5 and distance < 5000:
                normalized_angle = ((angle + 180) % 360) - 180
                angle_bin = round(normalized_angle / self.angle_resolution) * self.angle_resolution
                distance_bin = round(distance / self.distance_resolution) * self.distance_resolution
                
                key = (angle_bin, distance_bin)
                seen_obstacles.add(key)
                
                current_confidence = self.obstacle_confidence.get(key, 0)
                self.obstacle_confidence[key] = min(
                    self.max_confidence, 
                    current_confidence + self.confidence_increment
                )
        
        # Faster decay for test mode
        obstacles_to_remove = []
        for key, confidence in list(self.obstacle_confidence.items()):
            if key not in seen_obstacles:
                new_confidence = confidence - self.confidence_decay  
                if new_confidence <= 0:
                    obstacles_to_remove.append(key)
                else:
                    self.obstacle_confidence[key] = new_confidence
        
        for key in obstacles_to_remove:
            del self.obstacle_confidence[key]
    
    def get_confident_obstacles(self):
        """Get confident obstacles for test visualization"""
        return [(angle, distance) for (angle, distance), confidence 
                in self.obstacle_confidence.items() 
                if confidence >= self.confidence_threshold]
    
    def get_display_obstacles(self):
        """Get obstacles for display"""
        with self.data_lock:
            return self.latest_obstacles.copy()
    
    def start(self):
        """Start the test LiDAR system"""
        self.running = True
        self.shutdown_flag[0] = False
        
        data_thread = threading.Thread(target=self.data_acquisition_thread, daemon=True)
        data_thread.start()
        self.threads.append(data_thread)
        
        return True
    
    def stop(self):
        """Stop the system gracefully"""
        self.running = False
        self.shutdown_flag[0] = True
        
        for thread in self.threads:
            thread.join(timeout=2.0)


class EnhancedFacialAnimationRestorer:
    """Enhanced facial animation restoration for test mode"""
    
    def __init__(self):
        self.restoration_attempts = 0
        self.max_restoration_attempts = 8
        
    def restore_resting_face_immediately(self, robot):
        """Immediately restore resting face for test mode exit"""
        restoration_success = False
        
        for attempt in range(self.max_restoration_attempts):
            try:
                # Primary restoration method
                if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                    facial_manager = robot.facial_animation_manager
                    
                    # Bring to front
                    facial_manager.bring_to_front()
                    time.sleep(0.1)
                    
                    # Ensure window is open
                    if not hasattr(facial_manager, 'display') or facial_manager.display is None:
                        facial_manager.open_window()
                        time.sleep(0.2)
                    
                    # Display resting face
                    if hasattr(facial_manager, 'resting_face_img') and facial_manager.resting_face_img:
                        if facial_manager.display:
                            facial_manager.display.blit(facial_manager.resting_face_img, (0, 0))
                            pygame.display.flip()
                            restoration_success = True
                            break
                
                # Fallback: reinitialize
                if not restoration_success and hasattr(robot, 'facial_animation_manager'):
                    try:
                        facial_manager = robot.facial_animation_manager
                        facial_manager.close_window()
                        time.sleep(0.1)
                        facial_manager.open_window()
                        time.sleep(0.2)
                        
                        if hasattr(facial_manager, 'resting_face_img'):
                            facial_manager.display.blit(facial_manager.resting_face_img, (0, 0))
                            pygame.display.flip()
                            restoration_success = True
                            break
                    except Exception:
                        continue
                
                time.sleep(0.1)
                
            except Exception:
                continue
        
        return restoration_success


class EnhancedStableLidarTest(MaxineBehavior):
    """
    Enhanced Stable LiDAR Test with comprehensive facial animation restoration
    Ensures proper transition back to IDLE mode with resting face displayed
    Now includes Head Align functionality to use recorded center position
    """
    
    def __init__(self):
        super().__init__("Enhanced Stable LiDAR Test with Head Align")
        
        # Blackboard setup
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
        self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("HEAD_CALIBRATED_CENTER", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("HeadAlignOffSet", access=py_trees.common.Access.READ)
        
        # Core components
        self.lidar_system = None
        self.screen = None
        self.head_tracker = None
        self.facial_restorer = EnhancedFacialAnimationRestorer()
        self.initialized = False
        
        # Head Align functionality
        self.head_align_center = None
        self.head_moved_to_center = False
        
        # Speech tracking to prevent multiple announcements
        self.idle_mode_announced = False
        
        # Test mode configuration
        self.calibration_mode = False
        self.calibration_complete = True
        self.calibrated_center_position = 0.0
        self.lidar_data_received = True
        self.calibration_instructions_spoken = True
        
        # Head tracking state
        self.current_head_angle = 0.0
        self.head_angle_lock = threading.Lock()
        
        # Display parameters
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.update_counter = 0
        self.display_update_rate = 3
        
        if not pygame.get_init():
            pygame.init()
        
        pygame.font.init()
    
    def get_head_align_center_position(self):
        """Get the recorded head center position from blackboard"""
        try:
            if self.blackboard.exists("HeadAlignOffSet"):
                center_position = self.blackboard.get("HeadAlignOffSet")
                if center_position is not None:
                    print(f"✅ Retrieved head align center position: {center_position}°")
                    return center_position
                else:
                    print("⚠️ HeadAlignOffSet exists but is None")
            else:
                print("⚠️ HeadAlignOffSet not found in blackboard")
        except Exception as e:
            print(f"❌ Error retrieving HeadAlignOffSet: {e}")
        
        # Default to 0 if not found
        print("ℹ️ Using default head center position: 0°")
        return 0.0
    
    def move_head_to_center_position(self, target_angle):
        """Move head to the recorded center position"""
        robot = self.get_robot()
        success = False
        
        # Try servo controller first (preferred method)
        if hasattr(robot, 'servo_controller') and robot.servo_controller:
            try:
                success = robot.servo_controller.move_to_angle(target_angle)
                if success:
                    print(f"✅ Head moved to center position {target_angle}° using servo controller")
                    # Wait for movement to complete
                    robot.servo_controller.wait_for_position(timeout=5.0)
                else:
                    print("❌ Failed to move head using servo controller")
            except Exception as e:
                print(f"❌ Error moving head with servo controller: {e}")
                success = False
        
        # Fallback to head_velocity_manager
        elif hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
            try:
                success = robot.head_velocity_manager.set_head_angle_degrees(target_angle, wait_for_completion=True)
                if success:
                    print(f"✅ Head moved to center position {target_angle}° using head velocity manager")
                else:
                    print("❌ Failed to move head using head velocity manager")
            except Exception as e:
                print(f"❌ Error moving head with head velocity manager: {e}")
                success = False
        
        # No head controller available
        else:
            print("❌ No head controller available for centering")
            success = False
        
        return success
    
    def initialize_components(self):
        """Initialize test components with enhanced setup and head alignment"""
        if self.initialized:
            return True
        
        try:
            # Get head align center position
            self.head_align_center = self.get_head_align_center_position()
            
            # Move head to center position at startup
            if self.head_align_center is not None:
                print(f"🎯 Moving head to aligned center position: {self.head_align_center}°")
                self.head_moved_to_center = self.move_head_to_center_position(self.head_align_center)
                
                if self.head_moved_to_center:
                    print("✅ Head successfully moved to aligned center position")
                else:
                    print("⚠️ Failed to move head to aligned center position")
            
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("MAXINE ENHANCED STABLE LIDAR TEST - HEAD ALIGNED")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            
            self.draw_clean_interface()
            pygame.display.flip()
            
            self.blackboard.set("HEAD_CENTER_POSITION", self.head_align_center)
            
            # Initialize head tracker
            robot = self.get_robot()
            if (hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager) or \
               (hasattr(robot, 'servo_controller') and robot.servo_controller):
                try:
                    from ...behaviors.lidarchase.HeadTracker import HeadTracker
                    self.head_tracker = HeadTracker(
                        robot.head_velocity_manager if hasattr(robot, 'head_velocity_manager') else None,
                        robot.servo_controller if hasattr(robot, 'servo_controller') else None
                    )
                    self.head_tracker.start_tracking()
                    self.head_tracker.set_manual_position(self.head_align_center)
                    self.current_head_angle = self.head_align_center
                except Exception:
                    self.head_tracker = None
            else:
                self.head_tracker = None
            
            self.initialized = True
            self.start_stable_lidar()
            
            return True
            
        except Exception as e:
            print(f"❌ Error initializing components: {e}")
            self.initialized = False
            return False
    
    def start_stable_lidar(self):
        """Start stable LiDAR system for testing"""
        try:
            if not self.lidar_system:
                self.lidar_system = UltraStableLidarTestSystem()
                success = self.lidar_system.start()
                if success:
                    self.blackboard.set("LIDAR_SYSTEM", self.lidar_system)
                    print("✅ LiDAR system started successfully")
                    time.sleep(3)
                else:
                    print("❌ Failed to start LiDAR system")
        except Exception as e:
            print(f"❌ Error starting LiDAR system: {e}")
    
    def draw_clean_interface(self):
        """Draw clean test interface"""
        self.screen.fill((0, 0, 0))
        self.draw_radar_grid()
        self.draw_robot()
    
    def draw_radar_grid(self):
        """Draw radar grid for test visualization"""
        # Range circles
        for distance in [1000, 2000, 3000, 4000, 5000, 6000]:
            radius = distance * self.scale // 1000
            if radius < min(self.center_x, self.center_y) - 50:
                line_width = 3 if distance == 6000 else 2
                color = (0, 150, 0) if distance < 6000 else (255, 255, 0)
                pygame.draw.circle(self.screen, color, (self.center_x, self.center_y), radius, line_width)
        
        # Angle lines
        for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
            display_angle_rad = math.radians(90 - angle)
            line_length = min(self.center_x, self.center_y) - 80
            end_x = self.center_x + int(line_length * math.cos(display_angle_rad))
            end_y = self.center_y - int(line_length * math.sin(display_angle_rad))
            line_width = 3 if angle % 90 == 0 else 1
            pygame.draw.line(self.screen, (0, 150, 0), (self.center_x, self.center_y), (end_x, end_y), line_width)
    
    def draw_robot(self):
        """Draw robot representation"""
        pygame.draw.circle(self.screen, (0, 255, 0), (self.center_x, self.center_y), 15, 3)
        arrow_end_x = self.center_x
        arrow_end_y = self.center_y - 30
        pygame.draw.line(self.screen, (0, 255, 0), (self.center_x, self.center_y), (arrow_end_x, arrow_end_y), 5)
    
    def draw_lidar_data(self, obstacles):
        """Draw LiDAR obstacles for testing"""
        if not obstacles:
            return 0
        
        drawn_count = 0
        for angle, distance in obstacles:
            if 100 < distance < 6000:
                distance_m = distance / 1000.0
                display_angle_rad = math.radians(90 - angle)
                
                x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
                y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
                
                if 0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height():
                    pygame.draw.circle(self.screen, (255, 0, 0), (x, y), 4)
                    drawn_count += 1
        
        return drawn_count
    
    def draw_info(self, scan_count):
        """Draw enhanced test mode information with head align status"""
        head_status = f"Head Aligned: {self.head_align_center}°" if self.head_moved_to_center else "Head Alignment: Failed"
        
        info_lines = [
            "ENHANCED LIDAR TEST MODE - HEAD ALIGNED",
            f"Obstacles Detected: {scan_count}",
            head_status,
            "Real-time LiDAR Testing",
            "ESC - Return to IDLE MODE with Head Centered"
        ]
        
        font = pygame.font.Font(None, 48)
        small_font = pygame.font.Font(None, 36)
        y_offset = 50
        
        for line in info_lines:
            if "ENHANCED LIDAR TEST MODE" in line:
                color = (255, 255, 0)
                text_surface = pygame.font.Font(None, 64).render(line, True, color)
            elif "Head Aligned" in line:
                color = (0, 255, 0) if self.head_moved_to_center else (255, 100, 100)
                text_surface = font.render(line, True, color)
            elif "ESC" in line:
                color = (255, 100, 100)
                text_surface = font.render(line, True, color)
            elif "Real-time" in line:
                color = (100, 255, 100)
                text_surface = small_font.render(line, True, color)
            else:
                color = (255, 255, 255)
                text_surface = font.render(line, True, color)
            
            self.screen.blit(text_surface, (50, y_offset))
            y_offset += 60

    def center_head_for_idle_mode(self):
        """Return head to center position for IDLE mode transition"""
        try:
            # Use the recorded head align center position
            if self.head_align_center is not None:
                print(f"🎯 Returning head to aligned center position: {self.head_align_center}°")
                success = self.move_head_to_center_position(self.head_align_center)
                if success:
                    print("✅ Head successfully returned to aligned center position")
                    return True
                else:
                    print("⚠️ Failed to return head to aligned center position")
            
            # Fallback to standard centering
            robot = self.get_robot()
            
            # Multiple centering methods for reliability
            centering_methods = [
                lambda: robot.servo_controller.center() if hasattr(robot, 'servo_controller') and robot.servo_controller else None,
                lambda: robot.head_velocity_manager.center_head() if hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager else None,
                lambda: robot.head_manager.center_head() if hasattr(robot, 'head_manager') and robot.head_manager else None,
                lambda: robot.center_head() if hasattr(robot, 'center_head') else None
            ]
            
            successful_centers = 0
            for method in centering_methods:
                try:
                    result = method()
                    if result is not None:
                        successful_centers += 1
                        time.sleep(0.5)  # Allow servo time to move
                except Exception:
                    continue
            
            return successful_centers > 0
                    
        except Exception as e:
            print(f"❌ Error centering head for IDLE mode: {e}")
            return False

    def stop_robot(self):
        """Stop robot movement"""
        try:
            robot = self.get_robot()
            
            # Stop using available velocity managers
            velocity_managers = []
            if hasattr(robot, 'direct_velocity_manager') and robot.direct_velocity_manager:
                velocity_managers.append(robot.direct_velocity_manager)
            if hasattr(robot, 'velocity_manager') and robot.velocity_manager:
                velocity_managers.append(robot.velocity_manager)
            
            for velocity_manager in velocity_managers:
                try:
                    stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                    velocity_manager.perform_action(stop_config)
                except Exception:
                    continue
                
        except Exception:
            pass

    def perform_enhanced_idle_mode_transition(self):
        """Perform enhanced transition to IDLE mode with comprehensive facial restoration"""
        try:
            robot = self.get_robot()
            
            # Step 1: Stop robot movement
            self.stop_robot()
            
            # Step 2: Center head for proper IDLE positioning (using head align center)
            head_centered = self.center_head_for_idle_mode()
            
            # Step 3: Stop LiDAR system
            if self.lidar_system:
                self.lidar_system.stop()
                self.lidar_system = None
            
            # Step 4: Clear pygame events
            if pygame.get_init():
                pygame.event.clear()
            
            # Step 5: IMMEDIATELY restore facial animation
            restoration_success = self.facial_restorer.restore_resting_face_immediately(robot)
            
            # Step 6: Announce mode transition ONLY ONCE
            if not self.idle_mode_announced:
                try:
                    if hasattr(robot, 'speech_manager') and robot.speech_manager:
                        robot.speech_manager.perform_action("IDLE MODE")
                        self.idle_mode_announced = True
                except Exception:
                    pass
            
            # Step 7: Additional verification
            time.sleep(0.5)
            
            # Step 8: Final verification of facial animation
            if restoration_success:
                try:
                    if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                        if hasattr(robot.facial_animation_manager, 'resting_face_img'):
                            robot.facial_animation_manager.display.blit(
                                robot.facial_animation_manager.resting_face_img, (0, 0)
                            )
                            pygame.display.flip()
                except Exception:
                    pass
            
            return restoration_success and head_centered
            
        except Exception:
            # Emergency fallback
            try:
                self.stop_robot()
                self.center_head_for_idle_mode()
                if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                    robot.facial_animation_manager.bring_to_front()
            except Exception:
                pass
            return False

    def update(self) -> Status:
        """Enhanced update with improved exit handling"""
        try:
            if not self.initialized:
                self.initialize_components()
                return Status.RUNNING
            
            self.update_counter += 1
            
            # Handle events with enhanced exit handling
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        # Enhanced IDLE mode transition
                        transition_success = self.perform_enhanced_idle_mode_transition()
                        
                        robot = self.get_robot()
                        robot.set_mode(RobotMode.IDLE)
                        return Status.SUCCESS
                elif event.type == pygame.QUIT:
                    # Enhanced IDLE mode transition
                    transition_success = self.perform_enhanced_idle_mode_transition()
                    
                    robot = self.get_robot()
                    robot.set_mode(RobotMode.IDLE)
                    return Status.SUCCESS
            
            # Display update
            should_update_display = (self.update_counter % self.display_update_rate == 0)
            
            if should_update_display:
                try:
                    if self.screen:
                        self.screen.fill((0, 0, 0))
                        self.draw_radar_grid()
                        self.draw_robot()
                        
                        # Get and display obstacles
                        obstacles = []
                        if self.lidar_system:
                            obstacles = self.lidar_system.get_display_obstacles()
                            if obstacles:
                                self.draw_lidar_data(obstacles)
                        
                        # Draw enhanced test info
                        self.draw_info(len(obstacles) if obstacles else 0)
                        
                        pygame.display.flip()
                        
                except Exception:
                    pass
            
            return Status.RUNNING
            
        except Exception:
            # Emergency handling
            try:
                self.stop_robot()
                self.perform_enhanced_idle_mode_transition()
            except Exception:
                pass
            
            return Status.RUNNING

    def terminate(self, new_status: Status):
        """Enhanced termination with comprehensive facial restoration"""
        try:
            # Stop robot and tracking
            self.stop_robot()
            
            if self.head_tracker:
                self.head_tracker.stop_tracking()
                self.head_tracker = None
            
            # Perform enhanced IDLE mode transition
            self.perform_enhanced_idle_mode_transition()
            
            # Clean up blackboard
            try:
                if self.blackboard.exists("LIDAR_SYSTEM"):
                    self.blackboard.unset("LIDAR_SYSTEM")
            except Exception:
                pass
            
            self.initialized = False
            
        except Exception:
            # Emergency cleanup
            try:
                self.center_head_for_idle_mode()
                robot = self.get_robot()
                if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                    robot.facial_animation_manager.bring_to_front()
            except:
                pass
        
        super().terminate(new_status)


# Export for module compatibility
StableLidarTest = EnhancedStableLidarTest
LidarTestBehavior = EnhancedStableLidarTest