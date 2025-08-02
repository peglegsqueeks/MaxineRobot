import py_trees
import pygame
import time
import threading
from py_trees.common import Status
from ..MaxineBehavior import MaxineBehavior
from ..utils import make_press_esc_to_exit_behavior, conditional_behavior
from ..conditions.KeyPressedCondition import KeyPressedCondition
from ...types.RobotModes import RobotMode
from ...types.KeyboardKey import KeyboardKey


class EnhancedFacialAnimationRestorer:
    """Enhanced facial animation restoration for Head Align mode"""
    
    def __init__(self):
        self.restoration_attempts = 0
        self.max_restoration_attempts = 8
        
    def restore_resting_face_immediately(self, robot):
        """Immediately restore resting face for Head Align mode exit"""
        restoration_success = False
        
        for attempt in range(self.max_restoration_attempts):
            try:
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


class HeadAlignRawServoMode(MaxineBehavior):
    """
    Enhanced Head Align Mode with raw servo data display
    Following the pattern from LidarTestBehavior
    """
    
    def __init__(self):
        super().__init__("Head Align Raw Servo Mode")
        
        # Blackboard setup
        self.blackboard.register_key("HeadAlignOffSet", access=py_trees.common.Access.WRITE)
        
        # Core components
        self.screen = None
        self.facial_restorer = EnhancedFacialAnimationRestorer()
        self.initialized = False
        
        # Speech tracking
        self.idle_mode_announced = False
        self.head_align_announced = False
        
        # Display parameters
        self.center_x = 0
        self.center_y = 0
        self.font = None
        self.large_font = None
        
        # Movement parameters
        self.increment = 0.05
        self.min_servo_pos = -0.98
        self.max_servo_pos = 0.98
        
        if not pygame.get_init():
            pygame.init()
        pygame.font.init()
    
    def initialize_components(self):
        """Initialize Head Align components following LidarTestBehavior pattern"""
        if self.initialized:
            return True
        
        try:
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("MAXINE HEAD ALIGN MODE - RAW SERVO DATA")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            
            # Initialize fonts
            self.font = pygame.font.Font(None, 48)
            self.large_font = pygame.font.Font(None, 64)
            
            self.draw_clean_interface()
            pygame.display.flip()
            
            self.initialized = True
            return True
            
        except Exception:
            self.initialized = False
            return False
    
    def draw_clean_interface(self):
        """Draw the Head Align interface"""
        self.screen.fill((0, 0, 0))
        
        robot = self.get_robot()
        
        # Get raw servo data
        current_servo_pos = 0.0
        target_servo_pos = 0.0
        recorded_center_value = None
        servo_available = False
        
        try:
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                servo_available = True
                current_servo_pos = robot.servo_controller.get_position()
                if hasattr(robot.servo_controller, 'target_position'):
                    target_servo_pos = robot.servo_controller.target_position
                else:
                    target_servo_pos = current_servo_pos
            
            # Get recorded center value from blackboard
            try:
                if self.blackboard.exists("HeadAlignOffSet"):
                    recorded_center_value = self.blackboard.get("HeadAlignOffSet")
            except:
                pass
                
        except:
            pass
        
        # Draw servo information
        y_offset = 50
        
        # Title in bright yellow
        title_text = self.large_font.render("HEAD ALIGN MODE - RAW SERVO DATA", True, (255, 255, 0))
        self.screen.blit(title_text, (50, y_offset))
        y_offset += 100
        
        # Servo availability status
        if servo_available:
            status_text = self.font.render("Servo Controller: ACTIVE", True, (0, 255, 0))
        else:
            status_text = self.font.render("Servo Controller: NOT AVAILABLE", True, (255, 0, 0))
        self.screen.blit(status_text, (50, y_offset))
        y_offset += 80
        
        # Current raw servo position
        current_text = self.font.render(f"Current Raw Servo: {current_servo_pos:.6f}", True, (255, 255, 255))
        self.screen.blit(current_text, (50, y_offset))
        y_offset += 60
        
        # Target servo position
        target_text = self.font.render(f"Target Raw Servo:  {target_servo_pos:.6f}", True, (0, 255, 255))
        self.screen.blit(target_text, (50, y_offset))
        y_offset += 60
        
        # Position difference
        diff = abs(current_servo_pos - target_servo_pos)
        if diff > 0.01:
            diff_color = (255, 255, 0)  # Yellow if moving
        else:
            diff_color = (0, 255, 0)    # Green if stable
        diff_text = self.font.render(f"Position Error:    {diff:.6f}", True, diff_color)
        self.screen.blit(diff_text, (50, y_offset))
        y_offset += 80
        
        # Recorded center value
        if recorded_center_value is not None:
            center_color = (0, 255, 0)  # Green
            center_text = self.font.render(f"Recorded Center:   {recorded_center_value:.6f}", True, center_color)
        else:
            center_color = (255, 100, 100)  # Light red
            center_text = self.font.render("Recorded Center:   NOT SET", True, center_color)
        self.screen.blit(center_text, (50, y_offset))
        y_offset += 100
        
        # Control instructions
        instructions = [
            "CONTROLS:",
            "",
            "O - Move Head Left  (Raw Servo +0.05)",
            "P - Move Head Right (Raw Servo -0.05)",
            "C - Record Current Raw Servo as Center",
            "",
            "ESC - Return to IDLE MODE"
        ]
        
        for instruction in instructions:
            if instruction == "CONTROLS:":
                color = (255, 255, 0)  # Yellow header
                text = self.font.render(instruction, True, color)
                self.screen.blit(text, (50, y_offset))
                y_offset += 50
            elif "ESC" in instruction:
                color = (255, 100, 100)  # Light red for exit
                text = self.font.render(instruction, True, color)
                self.screen.blit(text, (50, y_offset))
                y_offset += 50
            elif "Record" in instruction:
                color = (100, 255, 100)  # Light green for record
                text = self.font.render(instruction, True, color)
                self.screen.blit(text, (50, y_offset))
                y_offset += 50
            elif instruction == "":
                y_offset += 25  # Small gap for empty lines
            else:
                color = (200, 200, 200)  # Light gray for movement
                text = self.font.render(instruction, True, color)
                self.screen.blit(text, (50, y_offset))
                y_offset += 50

    def perform_enhanced_idle_mode_transition(self):
        """Enhanced transition back to IDLE mode with facial restoration"""
        try:
            robot = self.get_robot()
            
            # Step 1: Cleanup pygame display
            try:
                if self.screen:
                    self.screen.fill((0, 0, 0))
                    pygame.display.flip()
            except Exception:
                pass
            
            # Step 2: Restore facial animation
            restoration_success = self.facial_restorer.restore_resting_face_immediately(robot)
            
            # Step 3: Announce mode transition ONLY ONCE
            if not self.idle_mode_announced:
                try:
                    if hasattr(robot, 'speech_manager') and robot.speech_manager:
                        robot.speech_manager.perform_action("IDLE MODE")
                        self.idle_mode_announced = True
                except Exception:
                    pass
            
            return restoration_success
            
        except Exception:
            return False

    def update(self) -> Status:
        """Main update loop following LidarTestBehavior pattern"""
        try:
            if not self.initialized:
                if not self.initialize_components():
                    return Status.RUNNING
                
                # Announce Head Align mode once
                if not self.head_align_announced:
                    robot = self.get_robot()
                    if hasattr(robot, 'speech_manager') and robot.speech_manager:
                        robot.speech_manager.perform_action("Head Align Mode")
                    self.head_align_announced = True
                
                return Status.RUNNING
            
            # Handle pygame events
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        # Enhanced IDLE mode transition
                        self.perform_enhanced_idle_mode_transition()
                        robot = self.get_robot()
                        robot.set_mode(RobotMode.IDLE)
                        return Status.SUCCESS
                    elif event.key == pygame.K_o:
                        # Move head left (raw servo +)
                        self.move_head_left()
                    elif event.key == pygame.K_p:
                        # Move head right (raw servo -)
                        self.move_head_right()
                    elif event.key == pygame.K_c:
                        # Record current raw servo as center
                        self.record_raw_servo_center()
                elif event.type == pygame.QUIT:
                    self.perform_enhanced_idle_mode_transition()
                    robot = self.get_robot()
                    robot.set_mode(RobotMode.IDLE)
                    return Status.SUCCESS
            
            # Update display
            self.draw_clean_interface()
            pygame.display.flip()
            
            return Status.RUNNING
            
        except Exception:
            return Status.FAILURE

    def move_head_left(self):
        """Move head left using raw servo position"""
        robot = self.get_robot()
        try:
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                current_pos = robot.servo_controller.get_position()
                new_pos = current_pos + self.increment
                if new_pos <= self.max_servo_pos:
                    robot.servo_controller.set_position(new_pos)
        except:
            pass

    def move_head_right(self):
        """Move head right using raw servo position"""
        robot = self.get_robot()
        try:
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                current_pos = robot.servo_controller.get_position()
                new_pos = current_pos - self.increment
                if new_pos >= self.min_servo_pos:
                    robot.servo_controller.set_position(new_pos)
        except:
            pass

    def record_raw_servo_center(self):
        """Record current raw servo position as center"""
        robot = self.get_robot()
        try:
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                current_raw_servo = robot.servo_controller.get_position()
                self.blackboard.set("HeadAlignOffSet", current_raw_servo)
                
                if hasattr(robot, 'speech_manager') and robot.speech_manager:
                    robot.speech_manager.perform_action("Head Center Recorded")
        except:
            pass


def make_head_align_sub_tree():
    """
    Returns the head align mode subtree focused on raw servo data
    Following the pattern from other working modes
    """
    # The main Head Align behavior that manages everything
    head_align_mode = HeadAlignRawServoMode()
    
    # Return just the main behavior - it handles everything internally
    return head_align_mode