from py_trees.common import Status

from src.types.HeadMovementDirection import HeadMovementDirection
from ..action_managers.VelocityManager import VelocityConfig
from .MaxineBehavior import MaxineBehavior


class HeadTurn(MaxineBehavior):
    """
    Moves the robot head in a particular direction using PID-controlled smooth movement
    Enhanced to work with the new servo system that protects the 4kg head assembly
    """

    def __init__(self, direction: HeadMovementDirection, wait_for_completion=False):
        """
        Initialises the direction

        arguments:
            - direction: the direction to move in
            - wait_for_completion: whether to wait for movement to complete before returning SUCCESS
        """
        super().__init__(name=f"Move head in {direction.value} direction (PID)")
        self.direction = direction
        self.wait_for_completion = wait_for_completion

    def update(self) -> Status:
        robot = self.get_robot()
        
        # Try the new servo controller first (preferred method)
        if hasattr(robot, 'servo_controller') and robot.servo_controller:
            try:
                if self.direction == HeadMovementDirection.LEFT:
                    success = robot.servo_controller.move_left()
                elif self.direction == HeadMovementDirection.RIGHT:
                    success = robot.servo_controller.move_right()
                elif self.direction == HeadMovementDirection.NONE:
                    success = True  # No movement needed
                else:
                    return Status.FAILURE
                
                if not success:
                    return Status.FAILURE
                
                # Optionally wait for movement to complete
                if self.wait_for_completion and self.direction != HeadMovementDirection.NONE:
                    success = robot.servo_controller.wait_for_position(timeout=8.0)
                    return Status.SUCCESS if success else Status.FAILURE
                
                return Status.SUCCESS
                
            except Exception as e:
                print(f"❌ Error in servo controller head movement: {e}")
                return Status.FAILURE
        
        # Fallback to head_velocity_manager (updated version with PID)
        elif hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
            try:
                success = robot.head_velocity_manager.perform_action(self.direction)
                
                if not success:
                    return Status.FAILURE
                
                # Optionally wait for movement to complete
                if self.wait_for_completion and self.direction != HeadMovementDirection.NONE:
                    success = robot.head_velocity_manager.wait_for_movement_complete(timeout=8.0)
                    return Status.SUCCESS if success else Status.FAILURE
                
                return Status.SUCCESS
                
            except Exception as e:
                print(f"❌ Error in head velocity manager: {e}")
                return Status.FAILURE
        
        # Legacy fallback to head_move_manager
        elif hasattr(robot, 'head_move_manager') and robot.head_move_manager:
            try:
                if hasattr(robot.head_move_manager, 'perform_action'):
                    robot.head_move_manager.perform_action(self.direction)
                else:
                    # Very old interface
                    if self.direction == HeadMovementDirection.LEFT:
                        robot.head_move_manager.move_left()
                    elif self.direction == HeadMovementDirection.RIGHT:
                        robot.head_move_manager.move_right()
                
                return Status.SUCCESS
                
            except Exception as e:
                print(f"❌ Error in legacy head move manager: {e}")
                return Status.FAILURE
        
        # No head controller available
        print("❌ No head controller available")
        return Status.FAILURE


class HeadTurnToAngle(MaxineBehavior):
    """
    New behavior to turn head to a specific angle using PID control
    Provides precise positioning for advanced behaviors
    """

    def __init__(self, target_angle_degrees: float, wait_for_completion=True):
        """
        Initialize the behavior

        arguments:
            - target_angle_degrees: Target angle in degrees (-90 to +90)
            - wait_for_completion: Whether to wait for movement to complete
        """
        super().__init__(name=f"Turn head to {target_angle_degrees:.1f}° (PID)")
        self.target_angle = target_angle_degrees
        self.wait_for_completion = wait_for_completion

    def update(self) -> Status:
        robot = self.get_robot()

        # Check angle bounds
        if self.target_angle < -90 or self.target_angle > 90:
            print(f"❌ Invalid head angle: {self.target_angle:.1f}° (must be -90 to +90)")
            return Status.FAILURE

        # Try the new servo controller first
        if hasattr(robot, 'servo_controller') and robot.servo_controller:
            try:
                success = robot.servo_controller.move_to_angle(self.target_angle)
                
                if not success:
                    return Status.FAILURE
                
                if self.wait_for_completion:
                    success = robot.servo_controller.wait_for_position(timeout=10.0)
                    return Status.SUCCESS if success else Status.FAILURE
                
                return Status.SUCCESS
                
            except Exception as e:
                print(f"❌ Error in servo controller angle movement: {e}")
                return Status.FAILURE

        # Fallback to head_velocity_manager
        elif hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
            try:
                success = robot.head_velocity_manager.set_head_angle_degrees(
                    self.target_angle, 
                    wait_for_completion=self.wait_for_completion
                )
                return Status.SUCCESS if success else Status.FAILURE
                
            except Exception as e:
                print(f"❌ Error in head velocity manager angle movement: {e}")
                return Status.FAILURE

        # No compatible head controller available
        print("❌ No compatible head controller available for angle positioning")
        return Status.FAILURE


class HeadScanBehavior(MaxineBehavior):
    """
    New behavior to perform a smooth head scanning pattern
    Useful for environmental awareness and search behaviors
    """

    def __init__(self, angle_range=60.0, steps=5, dwell_time=1.0):
        """
        Initialize the scanning behavior

        arguments:
            - angle_range: Total angle range in degrees to scan
            - steps: Number of positions to stop at during scan
            - dwell_time: Time to pause at each position
        """
        super().__init__(name=f"Head scan ±{angle_range/2:.1f}° in {steps} steps")
        self.angle_range = angle_range
        self.steps = steps
        self.dwell_time = dwell_time

    def update(self) -> Status:
        robot = self.get_robot()

        # Try head_velocity_manager first (has scan functionality)
        if hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
            try:
                success = robot.head_velocity_manager.scan_left_right(
                    angle_range=self.angle_range,
                    steps=self.steps,
                    dwell_time=self.dwell_time
                )
                return Status.SUCCESS if success else Status.FAILURE
                
            except Exception as e:
                print(f"❌ Error in head scan: {e}")
                return Status.FAILURE

        # Fallback: manual scan using servo controller
        elif hasattr(robot, 'servo_controller') and robot.servo_controller:
            try:
                print(f"🔄 Manual head scan: ±{self.angle_range/2:.1f}° in {self.steps} steps")
                
                # Calculate positions
                half_range = self.angle_range / 2
                if self.steps <= 1:
                    positions = [0]
                else:
                    positions = []
                    for i in range(self.steps):
                        angle = -half_range + (i * self.angle_range / (self.steps - 1))
                        positions.append(angle)
                
                # Perform scan
                for i, angle in enumerate(positions):
                    print(f"🎯 Scan position {i+1}/{self.steps}: {angle:.1f}°")
                    
                    if not robot.servo_controller.move_to_angle(angle):
                        print(f"⚠️ Failed to move to {angle:.1f}°")
                        continue
                    
                    if not robot.servo_controller.wait_for_position(timeout=8.0):
                        print(f"⚠️ Timeout waiting for position {angle:.1f}°")
                    
                    if self.dwell_time > 0:
                        import time
                        time.sleep(self.dwell_time)
                
                # Return to center
                print("🎯 Returning to center")
                robot.servo_controller.move_to_angle(0.0)
                robot.servo_controller.wait_for_position(timeout=8.0)
                
                return Status.SUCCESS
                
            except Exception as e:
                print(f"❌ Error in manual head scan: {e}")
                return Status.FAILURE

        print("❌ No head controller available for scanning")
        return Status.FAILURE


class HeadCenterBehavior(MaxineBehavior):
    """
    Behavior to center the head smoothly using PID control
    """

    def __init__(self, wait_for_completion=True):
        """
        Initialize the centering behavior

        arguments:
            - wait_for_completion: Whether to wait for centering to complete
        """
        super().__init__(name="Center head (PID)")
        self.wait_for_completion = wait_for_completion

    def update(self) -> Status:
        robot = self.get_robot()

        # Try head_velocity_manager first
        if hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
            try:
                success = robot.head_velocity_manager.center_head()
                return Status.SUCCESS if success else Status.FAILURE
                
            except Exception as e:
                print(f"❌ Error centering head: {e}")
                return Status.FAILURE

        # Fallback to servo controller
        elif hasattr(robot, 'servo_controller') and robot.servo_controller:
            try:
                success = robot.servo_controller.center()
                
                if not success:
                    return Status.FAILURE
                
                if self.wait_for_completion:
                    success = robot.servo_controller.wait_for_position(timeout=8.0)
                    return Status.SUCCESS if success else Status.FAILURE
                
                return Status.SUCCESS
                
            except Exception as e:
                print(f"❌ Error centering head with servo controller: {e}")
                return Status.FAILURE

        print("❌ No head controller available for centering")
        return Status.FAILURE