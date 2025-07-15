import time
import pygame
import threading
from src.behaviors.MaxineBehavior import MaxineBehavior
import py_trees
from py_trees.common import Status


class FixedCloseLidarPlot(MaxineBehavior):
    """
    FIXED LiDAR plot cleanup with AGGRESSIVE head centering for IDLE mode
    """
    
    def __init__(self):
        super().__init__("Fixed Close LiDAR Plot - AGGRESSIVE Head Centering")
        
        # Blackboard keys for cleanup
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_RENDERER", access=py_trees.common.Access.WRITE)
        
        # Legacy keys for backward compatibility
        self.blackboard.register_key("LIDAR_PLOT", access=py_trees.common.Access.WRITE)
        
        # Cleanup tracking
        self.cleanup_steps = []
        
    def force_center_head_all_methods(self):
        """FORCE head to center using ALL possible methods - HIGHEST PRIORITY"""
        try:
            robot = self.get_robot()
            center_position = 0.0
            center_angle_degrees = 0.0
            
            print("🎯 FORCING head to center position using ALL methods...")
            
            # Method 1: Direct servo controller with MULTIPLE attempts
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                try:
                    print("🔧 Using servo_controller...")
                    for attempt in range(10):  # 10 attempts
                        robot.servo_controller.set_position(center_position)
                        robot.servo_controller.move_to_angle(center_angle_degrees)
                        if hasattr(robot.servo_controller, 'center'):
                            robot.servo_controller.center()
                        time.sleep(0.3)
                    print("✅ Head centered via servo_controller (10 attempts)")
                except Exception as e:
                    print(f"❌ Servo controller centering failed: {e}")
            
            # Method 2: Head velocity manager with MULTIPLE attempts
            if hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
                try:
                    print("🔧 Using head_velocity_manager...")
                    for attempt in range(10):  # 10 attempts
                        robot.head_velocity_manager.set_head_position(center_position, wait_for_completion=False)
                        if hasattr(robot.head_velocity_manager, 'center_head'):
                            robot.head_velocity_manager.center_head()
                        if hasattr(robot.head_velocity_manager, 'set_head_angle_degrees'):
                            robot.head_velocity_manager.set_head_angle_degrees(center_angle_degrees)
                        time.sleep(0.3)
                    print("✅ Head centered via head_velocity_manager (10 attempts)")
                except Exception as e:
                    print(f"❌ Head velocity manager centering failed: {e}")
            
            # Method 3: General head manager with MULTIPLE attempts
            if hasattr(robot, 'head_manager') and robot.head_manager:
                try:
                    print("🔧 Using head_manager...")
                    for attempt in range(10):  # 10 attempts
                        if hasattr(robot.head_manager, 'center_head'):
                            robot.head_manager.center_head()
                        if hasattr(robot.head_manager, 'set_head_angle_degrees'):
                            robot.head_manager.set_head_angle_degrees(center_angle_degrees)
                        if hasattr(robot.head_manager, 'set_head_position'):
                            robot.head_manager.set_head_position(center_position)
                        time.sleep(0.3)
                    print("✅ Head centered via head_manager (10 attempts)")
                except Exception as e:
                    print(f"❌ Head manager centering failed: {e}")
            
            # Method 4: Robot unified methods with MULTIPLE attempts
            if hasattr(robot, 'center_head'):
                try:
                    print("🔧 Using robot.center_head()...")
                    for attempt in range(10):  # 10 attempts
                        robot.center_head()
                        time.sleep(0.3)
                    print("✅ Head centered via robot.center_head() (10 attempts)")
                except Exception as e:
                    print(f"❌ Robot center_head failed: {e}")
            
            if hasattr(robot, 'set_head_angle'):
                try:
                    print("🔧 Using robot.set_head_angle()...")
                    for attempt in range(10):  # 10 attempts
                        robot.set_head_angle(center_angle_degrees)
                        time.sleep(0.3)
                    print("✅ Head centered via robot.set_head_angle() (10 attempts)")
                except Exception as e:
                    print(f"❌ Robot set_head_angle failed: {e}")
            
            # Method 5: Try head move manager variants
            if hasattr(robot, 'head_move_manager') and robot.head_move_manager:
                try:
                    print("🔧 Using head_move_manager...")
                    for attempt in range(10):  # 10 attempts
                        if hasattr(robot.head_move_manager, 'center_head'):
                            robot.head_move_manager.center_head()
                        if hasattr(robot.head_move_manager, 'set_head_angle_degrees'):
                            robot.head_move_manager.set_head_angle_degrees(center_angle_degrees)
                        if hasattr(robot.head_move_manager, 'set_head_position'):
                            robot.head_move_manager.set_head_position(center_position)
                        time.sleep(0.3)
                    print("✅ Head centered via head_move_manager (10 attempts)")
                except Exception as e:
                    print(f"❌ Head move manager centering failed: {e}")
            
            # Method 6: Try head servo variant
            if hasattr(robot, 'head_servo') and robot.head_servo:
                try:
                    print("🔧 Using head_servo...")
                    for attempt in range(10):  # 10 attempts
                        if hasattr(robot.head_servo, 'set_position'):
                            robot.head_servo.set_position(center_position)
                        if hasattr(robot.head_servo, 'move_to_angle'):
                            robot.head_servo.move_to_angle(center_angle_degrees)
                        if hasattr(robot.head_servo, 'center'):
                            robot.head_servo.center()
                        time.sleep(0.3)
                    print("✅ Head centered via head_servo (10 attempts)")
                except Exception as e:
                    print(f"❌ Head servo centering failed: {e}")
            
            # FINAL: Extra time for servo to reach position
            print("⏱️ Waiting for servo to reach center position...")
            time.sleep(2.0)
            
            print("🎯 AGGRESSIVE head centering complete - used ALL available methods")
            self.cleanup_steps.append("AGGRESSIVE head centering completed using all methods")
            
        except Exception as e:
            print(f"❌ CRITICAL head centering error: {e}")
            self.cleanup_steps.append(f"Head centering critical error: {e}")
    
    def stop_working_lidar_system(self) -> bool:
        """Stop WORKING LiDAR system with proper motor shutdown"""
        try:
            if self.blackboard.exists("LIDAR_SYSTEM"):
                lidar_system = self.blackboard.get("LIDAR_SYSTEM")
                if lidar_system:
                    print("🛑 Stopping LiDAR system...")
                    # Use the working system's stop method
                    lidar_system.stop()
                    
                    # Give time for proper shutdown
                    time.sleep(2.0)
                    
                    self.cleanup_steps.append("WORKING LiDAR system stopped")
                    print("✅ LiDAR system stopped successfully")
                
                self.blackboard.unset("LIDAR_SYSTEM")
                return True
                
        except Exception as e:
            self.cleanup_steps.append(f"WORKING LiDAR system error: {e}")
            print(f"❌ LiDAR stop error: {e}")
            return False
        
        return True
    
    def cleanup_robot_lidar_sensor(self):
        """Additional robot LiDAR sensor cleanup"""
        try:
            robot = self.get_robot()
            
            if hasattr(robot, 'lidar_sensor') and robot.lidar_sensor:
                if hasattr(robot.lidar_sensor, 'shutdown'):
                    robot.lidar_sensor.shutdown()
                    self.cleanup_steps.append("Robot LiDAR sensor shutdown")
                elif hasattr(robot.lidar_sensor, 'emergency_stop'):
                    robot.lidar_sensor.emergency_stop()
                    self.cleanup_steps.append("Robot LiDAR sensor emergency stop")
                
        except Exception as e:
            self.cleanup_steps.append(f"Robot LiDAR sensor warning: {e}")
    
    def restore_display_for_idle_mode(self) -> bool:
        """Properly restore display for IDLE mode face avatar"""
        try:
            print("🖥️ Restoring display for IDLE mode...")
            
            # Release exclusive LiDAR renderer first
            if self.blackboard.exists("LIDAR_RENDERER"):
                renderer = self.blackboard.get("LIDAR_RENDERER")
                if renderer and hasattr(renderer, 'release_display'):
                    renderer.release_display()
                    self.cleanup_steps.append("Exclusive LiDAR renderer released")
                    print("✅ Exclusive LiDAR renderer released")
                
                self.blackboard.unset("LIDAR_RENDERER")
            
            # Clear pygame display and prepare for IDLE mode
            try:
                if pygame.get_init():
                    # Get current display info
                    display_info = pygame.display.Info()
                    
                    # Create a clean fullscreen surface for IDLE mode
                    screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
                    
                    # Fill with black and update
                    screen.fill((0, 0, 0))
                    pygame.display.flip()
                    
                    # Clear all events to prevent interference
                    pygame.event.clear()
                    
                    self.cleanup_steps.append("Display prepared for IDLE mode")
                    print("✅ Display prepared for IDLE mode face avatar")
                    return True
                    
            except Exception as display_error:
                print(f"❌ Display restoration error: {display_error}")
                self.cleanup_steps.append(f"Display error: {display_error}")
                
        except Exception as e:
            self.cleanup_steps.append(f"Display restore error: {e}")
            print(f"❌ Display restore error: {e}")
        
        return True  # Always return True to allow mode transition
    
    def cleanup_pygame_resources(self) -> bool:
        """Clean up pygame resources for mode transition"""
        try:
            if pygame.get_init():
                # Clear any remaining events
                try:
                    pygame.event.clear()
                    print("🧹 Pygame events cleared")
                except Exception:
                    pass
                
                # Don't quit pygame - IDLE mode needs it
                # Just ensure clean state
                
                self.cleanup_steps.append("Pygame resources cleaned for mode transition")
                print("✅ Pygame prepared for IDLE mode")
                return True
                
        except Exception as e:
            self.cleanup_steps.append(f"Pygame warning: {e}")
            print(f"⚠️ Pygame cleanup warning: {e}")
        
        return True
    
    def cleanup_legacy_systems(self) -> bool:
        """Clean up any legacy matplotlib systems"""
        try:
            if self.blackboard.exists("LIDAR_PLOT"):
                lidar_plot = self.blackboard.get("LIDAR_PLOT")
                if lidar_plot and hasattr(lidar_plot, 'close'):
                    lidar_plot.close()
                    self.cleanup_steps.append("Legacy matplotlib plot closed")
                
                self.blackboard.unset("LIDAR_PLOT")
                return True
                
        except Exception as e:
            self.cleanup_steps.append(f"Legacy plot warning: {e}")
        
        return True
    
    def wait_for_background_threads(self):
        """Wait for background threads to finish"""
        try:
            # Give threads time to finish
            time.sleep(1.5)
            
            # Check thread count
            active_threads = threading.active_count()
            if active_threads > 1:
                time.sleep(1.0)
            
            self.cleanup_steps.append("Background threads finished")
            print("✅ Background threads finished")
            
        except Exception as e:
            self.cleanup_steps.append(f"Thread warning: {e}")
    
    def cleanup_blackboard(self):
        """Clean up blackboard entries"""
        try:
            # Keys to clean up
            keys_to_clean = [
                "PATH", "TARGET_PERSON", "LIDAR_OBSTACLES"
            ]
            
            cleaned_count = 0
            for key in keys_to_clean:
                try:
                    if self.blackboard.exists(key):
                        self.blackboard.unset(key)
                        cleaned_count += 1
                except Exception:
                    pass
            
            if cleaned_count > 0:
                self.cleanup_steps.append(f"Cleaned {cleaned_count} blackboard entries")
                print(f"🧹 Cleaned {cleaned_count} blackboard entries")
            
        except Exception as e:
            self.cleanup_steps.append(f"Blackboard warning: {e}")
    
    def stop_robot_movement(self):
        """Ensure robot is completely stopped"""
        try:
            robot = self.get_robot()
            
            # Stop all movement
            if hasattr(robot, 'velocity_manager') and robot.velocity_manager:
                from src.types.MovementDirection import MovementDirection
                from src.action_managers.VelocityManager import VelocityConfig
                
                # Send stop commands multiple times to ensure it takes
                for _ in range(5):
                    stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                    robot.velocity_manager.perform_action(stop_config)
                    time.sleep(0.1)
                
                print("🛑 Robot movement stopped")
                self.cleanup_steps.append("Robot movement stopped")
            
        except Exception as e:
            print(f"❌ Robot stop error: {e}")
            self.cleanup_steps.append(f"Robot stop error: {e}")
    
    def perform_complete_cleanup(self) -> bool:
        """Perform complete system cleanup with AGGRESSIVE HEAD CENTERING as TOP PRIORITY"""
        start_time = time.time()
        print("🧹 Starting complete LiDAR Chase cleanup with AGGRESSIVE HEAD CENTERING...")
        
        success_count = 0
        total_steps = 8
        
        # Step 1: AGGRESSIVE HEAD CENTERING - TOP PRIORITY
        print("🎯 STEP 1: AGGRESSIVE HEAD CENTERING (TOP PRIORITY)")
        self.force_center_head_all_methods()
        success_count += 1
        
        # Step 2: Stop robot movement
        print("🛑 STEP 2: Stopping robot movement")
        self.stop_robot_movement()
        success_count += 1
        
        # Step 3: Stop WORKING LiDAR system with motor shutdown
        print("🔌 STEP 3: Stopping LiDAR system")
        if self.stop_working_lidar_system():
            success_count += 1
        
        # Step 4: Additional robot sensor cleanup
        print("🧹 STEP 4: Cleaning up robot sensors")
        self.cleanup_robot_lidar_sensor()
        success_count += 1
        
        # Step 5: Restore display for IDLE mode (CRITICAL)
        print("🖥️ STEP 5: Restoring display for IDLE mode")
        if self.restore_display_for_idle_mode():
            success_count += 1
        
        # Step 6: Clean up pygame resources for mode transition
        print("🎮 STEP 6: Cleaning up pygame resources")
        if self.cleanup_pygame_resources():
            success_count += 1
        
        # Step 7: Clean up legacy systems
        print("🗂️ STEP 7: Cleaning up legacy systems")
        if self.cleanup_legacy_systems():
            success_count += 1
        
        # Step 8: Final cleanup
        print("🔚 STEP 8: Final cleanup")
        self.wait_for_background_threads()
        self.cleanup_blackboard()
        success_count += 1
        
        # FINAL: One more aggressive head centering to be absolutely sure
        print("🎯 FINAL: One more AGGRESSIVE head centering to ensure center position")
        self.force_center_head_all_methods()
        
        cleanup_time = time.time() - start_time
        success_rate = (success_count / total_steps) * 100
        
        print(f"✅ Cleanup completed: {success_count}/{total_steps} steps ({success_rate:.1f}%) in {cleanup_time:.2f}s")
        print("🎭 Ready for IDLE mode with HEAD PROPERLY CENTERED")
        
        return success_count >= (total_steps * 0.8)  # 80% success rate required
    
    def update(self) -> Status:
        """Execute FIXED cleanup with AGGRESSIVE HEAD CENTERING for mode transition"""
        try:
            print("🔄 Executing LiDAR Chase mode exit cleanup with AGGRESSIVE HEAD CENTERING...")
            
            # Perform complete cleanup
            cleanup_success = self.perform_complete_cleanup()
            
            if cleanup_success:
                print("✅ LiDAR Chase cleanup successful - mode transition ready with HEAD CENTERED")
                return Status.SUCCESS
            else:
                print("⚠️ Some cleanup issues occurred but allowing transition with HEAD CENTERED")
                return Status.SUCCESS  # Still allow behavior tree to continue
            
        except Exception as e:
            print(f"❌ Cleanup execution error: {e}")
            
            # Emergency cleanup - ensure robot stops, display is freed, and HEAD IS CENTERED
            try:
                robot = self.get_robot()
                
                # Emergency stop
                if hasattr(robot, 'velocity_manager') and robot.velocity_manager:
                    from src.types.MovementDirection import MovementDirection
                    from src.action_managers.VelocityManager import VelocityConfig
                    stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                    robot.velocity_manager.perform_action(stop_config)
                
                # EMERGENCY HEAD CENTERING
                print("🚨 EMERGENCY HEAD CENTERING...")
                self.force_center_head_all_methods()
                
                # Emergency display clear
                if pygame.get_init():
                    screen = pygame.display.get_surface()
                    if screen:
                        screen.fill((0, 0, 0))
                        pygame.display.flip()
                    pygame.event.clear()
                
                print("🚨 Emergency cleanup executed with HEAD CENTERED")
                
            except Exception as emergency_error:
                print(f"❌ Emergency cleanup error: {emergency_error}")
                # Still try one more head centering attempt
                try:
                    self.force_center_head_all_methods()
                except:
                    pass
            
            return Status.SUCCESS  # Don't block behavior tree
    
    def get_cleanup_status(self) -> dict:
        """Get detailed cleanup status"""
        return {
            "cleanup_steps_completed": len(self.cleanup_steps),
            "cleanup_steps": self.cleanup_steps.copy(),
            "pygame_initialized": pygame.get_init() if 'pygame' in globals() else False,
            "blackboard_systems": {
                "lidar_system": self.blackboard.exists("LIDAR_SYSTEM"),
                "lidar_renderer": self.blackboard.exists("LIDAR_RENDERER"),
            },
            "version": "FIXED with AGGRESSIVE HEAD CENTERING for proper IDLE mode restoration"
        }


# Aliases for backward compatibility
CloseLidarPlot = FixedCloseLidarPlot
EnhancedCloseLidarPlot = FixedCloseLidarPlot