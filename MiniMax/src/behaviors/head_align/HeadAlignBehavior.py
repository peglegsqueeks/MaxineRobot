import py_trees
import pygame
from py_trees.common import Status

from ...behaviors.MaxineBehavior import MaxineBehavior
from ...behaviors.utils import make_press_esc_to_exit_behavior, conditional_behavior
from ...behaviors.conditions.KeyPressedCondition import KeyPressedCondition
from ...types.RobotModes import RobotMode
from ...types.KeyboardKey import KeyboardKey


class HeadAlignAnnounce(MaxineBehavior):
    """
    Announces "HEAD ALIGN" when the mode is first entered
    """

    def __init__(self):
        super().__init__("Announce HEAD ALIGN")
        self.announced = False

    def update(self) -> Status:
        if not self.announced:
            robot = self.get_robot()
            robot.speech_manager.perform_action("HEAD ALIGN")
            self.announced = True
            return Status.SUCCESS
        return Status.SUCCESS


class HeadAlignMove(MaxineBehavior):
    """
    Moves the head using the direct robot methods that are actually available
    Uses robot.set_head_angle() and robot.get_head_angle() methods
    """

    def __init__(self, direction: str):
        """
        Initialize head movement using direct robot methods
        
        arguments:
            - direction: 'left' or 'right'
        """
        super().__init__(f"Head align move {direction}")
        self.direction = direction
        self.increment = 5.0  # degrees per key press
        self.max_angle = 90.0
        self.min_angle = -90.0

    def update(self) -> Status:
        robot = self.get_robot()
        
        # Use the direct robot methods that are available
        try:
            # Get current angle using robot.get_head_angle()
            if hasattr(robot, 'get_head_angle') and callable(robot.get_head_angle):
                current_angle = robot.get_head_angle()
                print(f"✅ robot.get_head_angle() = {current_angle}")
            else:
                print("❌ robot.get_head_angle() not available")
                return Status.FAILURE
            
            # Calculate new angle based on direction
            if self.direction == 'left':
                new_angle = current_angle - self.increment
            elif self.direction == 'right':
                new_angle = current_angle + self.increment
            else:
                return Status.FAILURE
            
            # Enforce angle limits
            if new_angle < self.min_angle:
                new_angle = self.min_angle
            elif new_angle > self.max_angle:
                new_angle = self.max_angle
            
            # Only move if angle actually changed
            if abs(new_angle - current_angle) < 0.1:
                print(f"ℹ️ No movement needed: {new_angle} ≈ {current_angle}")
                return Status.SUCCESS
            
            # Move head to new angle using robot.set_head_angle()
            if hasattr(robot, 'set_head_angle') and callable(robot.set_head_angle):
                print(f"🎯 Moving head {self.direction}: {current_angle}° → {new_angle}°")
                result = robot.set_head_angle(new_angle)
                print(f"✅ robot.set_head_angle({new_angle}) = {result}")
                return Status.SUCCESS
            else:
                print("❌ robot.set_head_angle() not available")
                return Status.FAILURE
                
        except Exception as e:
            print(f"❌ Error using direct robot methods: {e}")
            return Status.FAILURE


class RecordHeadCenterPosition(MaxineBehavior):
    """
    Records the current head position as the center position using direct robot methods
    """

    def __init__(self):
        super().__init__("Record Head Center Position")
        
        # Register the HeadAlignOffSet key on the blackboard
        self.blackboard.register_key(
            "HeadAlignOffSet", access=py_trees.common.Access.WRITE
        )

    def update(self) -> Status:
        robot = self.get_robot()
        
        try:
            # Get current head angle using robot.get_head_angle()
            if hasattr(robot, 'get_head_angle') and callable(robot.get_head_angle):
                current_angle = robot.get_head_angle()
                print(f"✅ robot.get_head_angle() = {current_angle}")
                
                # Save to blackboard
                self.blackboard.set("HeadAlignOffSet", current_angle)
                
                # Announce that head is centered
                robot.speech_manager.perform_action("Head Centred")
                
                print(f"✅ Head center position recorded: {current_angle}° (saved to blackboard as 'HeadAlignOffSet')")
                
                return Status.SUCCESS
            else:
                print("❌ robot.get_head_angle() not available")
                return Status.FAILURE
                
        except Exception as e:
            print(f"❌ Error recording head position: {e}")
            return Status.FAILURE


class HeadAlignDisplay(MaxineBehavior):
    """
    Displays head angle and servo value on the pygame surface in yellow text
    """

    def __init__(self):
        super().__init__("Head Align Display")
        self.font = None

    def update(self) -> Status:
        robot = self.get_robot()
        
        # Get the pygame surface from the robot's display
        surface = None
        if hasattr(robot, 'screen') and robot.screen:
            surface = robot.screen
        elif hasattr(robot, 'display_surface') and robot.display_surface:
            surface = robot.display_surface
        else:
            # Try to get surface from pygame display
            try:
                surface = pygame.display.get_surface()
            except:
                return Status.SUCCESS  # No display available, skip
        
        if surface is None:
            return Status.SUCCESS
        
        # Initialize font if not already done
        if self.font is None:
            try:
                self.font = pygame.font.Font(None, 36)
            except:
                return Status.SUCCESS
        
        # Get current head angle using direct robot method
        head_angle = 0.0
        servo_value = 0.0
        
        try:
            if hasattr(robot, 'get_head_angle') and callable(robot.get_head_angle):
                head_angle = robot.get_head_angle()
            
            # Try to get servo value if available
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                try:
                    servo_value = robot.servo_controller.get_position()
                except:
                    pass
        except:
            pass
        
        # Create text surfaces in yellow
        try:
            angle_text = self.font.render(f"Head Angle: {head_angle:.1f}°", True, (255, 255, 0))  # Yellow
            servo_text = self.font.render(f"Servo Value: {servo_value:.3f}", True, (255, 255, 0))  # Yellow
            
            # Position text in upper left corner
            surface.blit(angle_text, (10, 10))
            surface.blit(servo_text, (10, 50))
            
        except Exception as e:
            # If text rendering fails, just continue
            pass
        
        return Status.SUCCESS


class HeadAlignIdle(MaxineBehavior):
    """
    Simple idle behavior that does nothing - just returns SUCCESS
    """

    def __init__(self):
        super().__init__("Head Align Idle")

    def update(self) -> Status:
        # This behavior does nothing - just returns SUCCESS
        return Status.SUCCESS


def make_head_align_sub_tree():
    """
    Returns the head align mode subtree using direct robot methods
    """
    # Exit to idle behavior when ESC is pressed
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
    
    # Announce HEAD ALIGN once when mode is entered
    announce_behavior = HeadAlignAnnounce()
    
    # Head movement behaviors using direct robot methods
    move_head_left = conditional_behavior(
        HeadAlignMove('left'), 
        KeyPressedCondition(KeyboardKey.O)
    )
    
    move_head_right = conditional_behavior(
        HeadAlignMove('right'), 
        KeyPressedCondition(KeyboardKey.P)
    )
    
    # Record head center position behavior
    record_center_position = conditional_behavior(
        RecordHeadCenterPosition(),
        KeyPressedCondition(KeyboardKey.C)
    )
    
    # Display head angle and servo value
    display_behavior = HeadAlignDisplay()
    
    # Idle behavior that does nothing
    idle_behavior = HeadAlignIdle()

    # Sub tree is a selector that will:
    # 1. Exit to idle mode if ESC is pressed
    # 2. Move head left if 'o' is pressed (using robot.set_head_angle)
    # 3. Move head right if 'p' is pressed (using robot.set_head_angle)
    # 4. Record head center position if 'c' is pressed (using robot.get_head_angle)
    # 5. Display head angle and servo value (using robot.get_head_angle)
    # 6. Announce HEAD ALIGN (runs once)
    # 7. Run idle behavior (does nothing)
    return py_trees.composites.Selector(
        "head align mode behavior",
        memory=True,
        children=[
            exit_mode_behavior,
            move_head_left,
            move_head_right,
            record_center_position,
            display_behavior,
            announce_behavior,
            idle_behavior,
        ]
    )