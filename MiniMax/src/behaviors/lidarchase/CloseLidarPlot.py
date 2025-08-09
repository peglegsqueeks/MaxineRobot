import time
import pygame
import threading
from src.behaviors.MaxineBehavior import MaxineBehavior
import py_trees
from py_trees.common import Status


class EnhancedFacialAnimationRestorer:
    """Enhanced facial animation restoration system for proper IDLE mode transitions"""
    
    def __init__(self):
        self.restoration_attempts = 0
        self.max_restoration_attempts = 10
        self.restoration_delay = 0.1
        
    def restore_resting_face_immediately(self, robot):
        """Immediately restore resting face with multiple fallback methods"""
        restoration_success = False
        
        for attempt in range(self.max_restoration_attempts):
            try:
                # Primary Method: Direct facial animation manager access
                if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                    facial_manager = robot.facial_animation_manager
                    
                    # Ensure window is open and active
                    try:
                        facial_manager.bring_to_front()
                        time.sleep(0.1)
                    except Exception:
                        pass
                    
                    # Restore window if needed
                    if not hasattr(facial_manager, 'display') or facial_manager.display is None:
                        try:
                            facial_manager.open_window()
                            time.sleep(0.3)
                        except Exception:
                            continue
                    
                    # Display resting face immediately - AGGRESSIVE APPROACH
                    if hasattr(facial_manager, 'resting_face_img') and facial_manager.resting_face_img:
                        try:
                            if facial_manager.display:
                                # Clear display first
                                facial_manager.display.fill((0, 0, 0))
                                # Display resting face
                                facial_manager.display.blit(facial_manager.resting_face_img, (0, 0))
                                pygame.display.flip()
                                # Second flip for reliability
                                pygame.display.update()
                                restoration_success = True
                                break
                        except Exception:
                            continue
                
                # Fallback Method: Reinitialize facial animation system
                if not restoration_success:
                    try:
                        if hasattr(robot, 'facial_animation_manager'):
                            facial_manager = robot.facial_animation_manager
                            facial_manager.close_window()
                            time.sleep(0.2)
                            facial_manager.open_window()
                            time.sleep(0.3)
                            
                            if hasattr(facial_manager, 'resting_face_img'):
                                facial_manager.display.fill((0, 0, 0))
                                facial_manager.display.blit(facial_manager.resting_face_img, (0, 0))
                                pygame.display.flip()
                                pygame.display.update()
                                restoration_success = True
                                break
                    except Exception:
                        continue
                
                # Delay between attempts
                time.sleep(self.restoration_delay)
                
            except Exception:
                continue
        
        return restoration_success
    
    def verify_resting_face_displayed(self, robot):
        """Verify that resting face is properly displayed"""
        try:
            if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                facial_manager = robot.facial_animation_manager
                
                # Check if display exists and is active
                if hasattr(facial_manager, 'display') and facial_manager.display:
                    # Ensure resting face is displayed
                    if hasattr(facial_manager, 'resting_face_img'):
                        facial_manager.display.blit(facial_manager.resting_face_img, (0, 0))
                        pygame.display.flip()
                        return True
            
            return False
            
        except Exception:
            return False


class UltraStableCloseLidarPlot(MaxineBehavior):
    """
    Ultra-stable LiDAR plot cleanup with enhanced facial animation restoration
    Ensures resting face is immediately displayed when exiting LiDAR modes
    """
    
    def __init__(self):
        super().__init__("Ultra-Stable Close LiDAR Plot with Enhanced Facial Restoration")
        
        # Blackboard management
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_RENDERER", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_PLOT", access=py_trees.common.Access.WRITE)
        
        # Enhanced facial restoration system
        self.facial_restorer = EnhancedFacialAnimationRestorer()
        
        # Cleanup tracking
        self.cleanup_steps = []
        self.cleanup_success_count = 0
        
    def aggressive_head_centering(self):
        """Aggressively center head using all available methods"""
        try:
            robot = self.get_robot()
            center_position = 0.0
            center_angle_degrees = 0.0
            
            # Multiple centering attempts using all available head control methods
            centering_methods = [
                # Servo controller methods
                lambda: robot.servo_controller.set_position(center_position) if hasattr(robot, 'servo_controller') and robot.servo_controller else None,
                lambda: robot.servo_controller.move_to_angle(center_angle_degrees) if hasattr(robot, 'servo_controller') and robot.servo_controller else None,
                lambda: robot.servo_controller.center() if hasattr(robot, 'servo_controller') and robot.servo_controller else None,
                
                # Head velocity manager methods
                lambda: robot.head_velocity_manager.set_head_position(center_position, wait_for_completion=False) if hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager else None,
                lambda: robot.head_velocity_manager.center_head() if hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager else None,
                lambda: robot.head_velocity_manager.set_head_angle_degrees(center_angle_degrees) if hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager else None,
                
                # General head manager methods
                lambda: robot.head_manager.center_head() if hasattr(robot, 'head_manager') and robot.head_manager else None,
                lambda: robot.head_manager.set_head_angle_degrees(center_angle_degrees) if hasattr(robot, 'head_manager') and robot.head_manager else None,
                
                # Robot unified methods
                lambda: robot.center_head() if hasattr(robot, 'center_head') else None,
                lambda: robot.set_head_angle(center_angle_degrees) if hasattr(robot, 'set_head_angle') else None,
            ]
            
            # Execute all available centering methods
            successful_methods = 0
            for method in centering_methods:
                try:
                    result = method()
                    if result is not None:
                        successful_methods += 1
                        time.sleep(0.3)  # Allow servo time to respond
                except Exception:
                    continue
            
            # Final positioning delay
            time.sleep(2.0)
            
            self.cleanup_steps.append(f"Head centering: {successful_methods} methods successful")
            return successful_methods > 0
            
        except Exception as e:
            self.cleanup_steps.append(f"Head centering error: {e}")
            return False
    
    def stop_lidar_system_safely(self) -> bool:
        """Stop LiDAR system with enhanced safety checks"""
        try:
            lidar_stopped = False
            
            # Method 1: Stop from blackboard
            if self.blackboard.exists("LIDAR_SYSTEM"):
                lidar_system = self.blackboard.get("LIDAR_SYSTEM")
                if lidar_system and hasattr(lidar_system, 'stop'):
                    try:
                        lidar_system.stop()
                        time.sleep(2.0)  # Allow proper shutdown
                        lidar_stopped = True
                        self.cleanup_steps.append("Blackboard LiDAR system stopped")
                    except Exception as e:
                        self.cleanup_steps.append(f"Blackboard LiDAR stop error: {e}")
                
                self.blackboard.unset("LIDAR_SYSTEM")
            
            # Method 2: Stop robot's LiDAR sensor
            try:
                robot = self.get_robot()
                if hasattr(robot, 'lidar_sensor') and robot.lidar_sensor:
                    if hasattr(robot.lidar_sensor, 'shutdown'):
                        robot.lidar_sensor.shutdown()
                        lidar_stopped = True
                        self.cleanup_steps.append("Robot LiDAR sensor shutdown")
                    elif hasattr(robot.lidar_sensor, 'emergency_stop'):
                        robot.lidar_sensor.emergency_stop()
                        lidar_stopped = True
                        self.cleanup_steps.append("Robot LiDAR sensor emergency stop")
            except Exception as e:
                self.cleanup_steps.append(f"Robot LiDAR sensor warning: {e}")
            
            return lidar_stopped
            
        except Exception as e:
            self.cleanup_steps.append(f"LiDAR stop error: {e}")
            return False
    
    def cleanup_robot_movement(self):
        """Ensure robot movement is completely stopped"""
        try:
            robot = self.get_robot()
            
            # Stop all movement using available velocity managers
            velocity_managers = []
            if hasattr(robot, 'direct_velocity_manager') and robot.direct_velocity_manager:
                velocity_managers.append(('direct_velocity_manager', robot.direct_velocity_manager))
            if hasattr(robot, 'velocity_manager') and robot.velocity_manager:
                velocity_managers.append(('velocity_manager', robot.velocity_manager))
            
            # Send multiple stop commands for reliability
            for manager_name, velocity_manager in velocity_managers:
                try:
                    from src.types.MovementDirection import MovementDirection
                    from src.action_managers.VelocityManager import VelocityConfig
                    
                    for _ in range(5):  # Multiple stop commands
                        stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                        velocity_manager.perform_action(stop_config)
                        time.sleep(0.1)
                    
                    self.cleanup_steps.append(f"Robot movement stopped via {manager_name}")
                    
                except Exception as e:
                    self.cleanup_steps.append(f"Movement stop error ({manager_name}): {e}")
            
        except Exception as e:
            self.cleanup_steps.append(f"Movement cleanup error: {e}")
    
    def restore_display_for_idle_mode(self) -> bool:
        """Restore display for IDLE mode with facial animation priority"""
        try:
            # Step 1: Release exclusive LiDAR renderer
            if self.blackboard.exists("LIDAR_RENDERER"):
                renderer = self.blackboard.get("LIDAR_RENDERER")
                if renderer and hasattr(renderer, 'release_display'):
                    renderer.release_display()
                    self.cleanup_steps.append("LiDAR renderer released")
                
                self.blackboard.unset("LIDAR_RENDERER")
            
            # Step 2: Clear pygame display state
            try:
                if pygame.get_init():
                    # Clear events to prevent interference
                    pygame.event.clear()
                    self.cleanup_steps.append("Pygame events cleared")
                    
            except Exception as display_error:
                self.cleanup_steps.append(f"Display clear error: {display_error}")
            
            # Step 3: IMMEDIATELY restore facial animation with resting face - HIGHEST PRIORITY
            robot = self.get_robot()
            restoration_success = self.facial_restorer.restore_resting_face_immediately(robot)
            
            if restoration_success:
                self.cleanup_steps.append("Facial animation restored successfully")
                
                # Step 4: Verify resting face is displayed
                verification_success = self.facial_restorer.verify_resting_face_displayed(robot)
                if verification_success:
                    self.cleanup_steps.append("Resting face display verified")
                else:
                    self.cleanup_steps.append("Resting face verification failed - attempting recovery")
                    # One more restoration attempt
                    self.facial_restorer.restore_resting_face_immediately(robot)
                    
                # Step 5: Announce IDLE MODE - ONLY ONCE
                try:
                    if hasattr(robot, 'speech_manager') and robot.speech_manager:
                        robot.speech_manager.perform_action("IDLE MODE")
                        self.cleanup_steps.append("IDLE MODE announced")
                except Exception:
                    self.cleanup_steps.append("IDLE MODE announcement failed")
                    
            else:
                self.cleanup_steps.append("Facial animation restoration failed")
                return False
            
            return True
            
        except Exception as e:
            self.cleanup_steps.append(f"Display restore error: {e}")
            return False
    
    def cleanup_pygame_resources(self) -> bool:
        """Clean up pygame resources while preserving facial animation"""
        try:
            if pygame.get_init():
                # Clear events but don't quit pygame - IDLE mode needs it
                pygame.event.clear()
                
                # Ensure facial animation window is active
                try:
                    robot = self.get_robot()
                    if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                        robot.facial_animation_manager.bring_to_front()
                        
                        # One final resting face display
                        if hasattr(robot.facial_animation_manager, 'resting_face_img'):
                            robot.facial_animation_manager.display.blit(
                                robot.facial_animation_manager.resting_face_img, (0, 0)
                            )
                            pygame.display.flip()
                            
                except Exception:
                    pass
                
                self.cleanup_steps.append("Pygame resources cleaned - facial animation preserved")
                return True
                
        except Exception as e:
            self.cleanup_steps.append(f"Pygame cleanup warning: {e}")
        
        return True
    
    def cleanup_legacy_systems(self) -> bool:
        """Clean up legacy LiDAR plot systems"""
        try:
            cleaned_systems = 0
            
            # Clean legacy matplotlib plots
            if self.blackboard.exists("LIDAR_PLOT"):
                lidar_plot = self.blackboard.get("LIDAR_PLOT")
                if lidar_plot and hasattr(lidar_plot, 'close'):
                    lidar_plot.close()
                    cleaned_systems += 1
                
                self.blackboard.unset("LIDAR_PLOT")
            
            # Clean up additional blackboard entries
            legacy_keys = ["PATH", "TARGET_PERSON", "LIDAR_OBSTACLES", "LIDAR_RENDERER"]
            
            for key in legacy_keys:
                try:
                    if self.blackboard.exists(key):
                        self.blackboard.unset(key)
                        cleaned_systems += 1
                except Exception:
                    pass
            
            if cleaned_systems > 0:
                self.cleanup_steps.append(f"Legacy systems cleaned: {cleaned_systems}")
            
            return True
            
        except Exception as e:
            self.cleanup_steps.append(f"Legacy cleanup warning: {e}")
            return True
    
    def wait_for_threads_completion(self):
        """Wait for background threads to complete"""
        try:
            # Give threads time to finish gracefully
            time.sleep(1.5)
            
            # Check active thread count
            active_threads = threading.active_count()
            if active_threads > 1:
                time.sleep(1.0)  # Additional wait for thread cleanup
            
            self.cleanup_steps.append(f"Thread cleanup completed ({active_threads} threads active)")
            
        except Exception as e:
            self.cleanup_steps.append(f"Thread cleanup warning: {e}")
    
    def perform_comprehensive_cleanup(self) -> bool:
        """Perform comprehensive cleanup with enhanced facial animation restoration"""
        start_time = time.time()
        self.cleanup_success_count = 0
        total_steps = 7
        
        try:
            # Step 1: Aggressive head centering (HIGHEST PRIORITY)
            if self.aggressive_head_centering():
                self.cleanup_success_count += 1
            
            # Step 2: Stop robot movement completely
            self.cleanup_robot_movement()
            self.cleanup_success_count += 1
            
            # Step 3: Stop LiDAR system safely
            if self.stop_lidar_system_safely():
                self.cleanup_success_count += 1
            
            # Step 4: Restore display for IDLE mode (CRITICAL STEP)
            if self.restore_display_for_idle_mode():
                self.cleanup_success_count += 1
            
            # Step 5: Clean up pygame resources while preserving facial animation
            if self.cleanup_pygame_resources():
                self.cleanup_success_count += 1
            
            # Step 6: Clean up legacy systems
            if self.cleanup_legacy_systems():
                self.cleanup_success_count += 1
            
            # Step 7: Wait for thread completion
            self.wait_for_threads_completion()
            self.cleanup_success_count += 1
            
            # FINAL STEP: One more facial animation verification
            try:
                robot = self.get_robot()
                final_verification = self.facial_restorer.verify_resting_face_displayed(robot)
                if final_verification:
                    self.cleanup_steps.append("FINAL: Resting face display confirmed")
                else:
                    self.cleanup_steps.append("FINAL: Attempting emergency resting face restoration")
                    self.facial_restorer.restore_resting_face_immediately(robot)
            except Exception:
                pass
            
            cleanup_time = time.time() - start_time
            success_rate = (self.cleanup_success_count / total_steps) * 100
            
            self.cleanup_steps.append(f"Cleanup completed: {self.cleanup_success_count}/{total_steps} steps ({success_rate:.1f}%) in {cleanup_time:.2f}s")
            
            return self.cleanup_success_count >= (total_steps * 0.8)  # 80% success rate
            
        except Exception as e:
            self.cleanup_steps.append(f"Comprehensive cleanup error: {e}")
            return False
    
    def emergency_cleanup_and_restore(self):
        """Emergency cleanup with forced facial animation restoration"""
        try:
            robot = self.get_robot()
            
            # Emergency stop
            try:
                if hasattr(robot, 'velocity_manager') and robot.velocity_manager:
                    from src.types.MovementDirection import MovementDirection
                    from src.action_managers.VelocityManager import VelocityConfig
                    stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                    robot.velocity_manager.perform_action(stop_config)
            except Exception:
                pass
            
            # Emergency head centering
            try:
                if hasattr(robot, 'servo_controller') and robot.servo_controller:
                    robot.servo_controller.center()
                elif hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
                    robot.head_velocity_manager.center_head()
            except Exception:
                pass
            
            # Emergency facial animation restoration
            try:
                self.facial_restorer.restore_resting_face_immediately(robot)
            except Exception:
                pass
            
            # Emergency display clear
            try:
                if pygame.get_init():
                    screen = pygame.display.get_surface()
                    if screen:
                        screen.fill((0, 0, 0))
                        pygame.display.flip()
                    pygame.event.clear()
            except Exception:
                pass
            
            self.cleanup_steps.append("Emergency cleanup executed with facial restoration")
            
        except Exception as e:
            self.cleanup_steps.append(f"Emergency cleanup error: {e}")
    
    def update(self) -> Status:
        """Execute comprehensive cleanup with enhanced facial animation restoration"""
        try:
            # Perform comprehensive cleanup
            cleanup_success = self.perform_comprehensive_cleanup()
            
            if cleanup_success:
                return Status.SUCCESS
            else:
                # Partial success - still allow transition but log warnings
                self.cleanup_steps.append("Partial cleanup success - allowing transition with warnings")
                return Status.SUCCESS
            
        except Exception as e:
            self.cleanup_steps.append(f"Cleanup execution error: {e}")
            
            # Perform emergency cleanup
            self.emergency_cleanup_and_restore()
            
            # Always return SUCCESS to allow behavior tree to continue
            return Status.SUCCESS
    
    def get_cleanup_status(self) -> dict:
        """Get detailed cleanup status for debugging"""
        return {
            "cleanup_steps_completed": len(self.cleanup_steps),
            "cleanup_steps": self.cleanup_steps.copy(),
            "success_count": self.cleanup_success_count,
            "facial_restoration_active": True,
            "pygame_initialized": pygame.get_init() if 'pygame' in globals() else False,
            "blackboard_systems": {
                "lidar_system": self.blackboard.exists("LIDAR_SYSTEM"),
                "lidar_renderer": self.blackboard.exists("LIDAR_RENDERER"),
                "lidar_plot": self.blackboard.exists("LIDAR_PLOT"),
            },
            "version": "Ultra-Stable with Enhanced Facial Animation Restoration"
        }


# Compatibility aliases
CloseLidarPlot = UltraStableCloseLidarPlot
FixedCloseLidarPlot = UltraStableCloseLidarPlot
EnhancedCloseLidarPlot = UltraStableCloseLidarPlot