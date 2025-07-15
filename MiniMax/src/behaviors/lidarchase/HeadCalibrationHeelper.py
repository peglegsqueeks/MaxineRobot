"""
Head Calibration Helper
Provides utilities for head calibration in LiDAR Chase mode
"""

import pygame
from typing import Dict, Any
from src.types.KeyboardKey import KeyboardKey


class HeadCalibrationKeyHandler:
    """
    Simplified keyboard handler for head calibration
    Integrates with existing keyboard sensor system
    """
    
    def __init__(self):
        self.keys_pressed = set()
        self.last_key_states = {}
        
    def update_key_states(self, keyboard_reading):
        """
        Update key states from keyboard reading
        
        Args:
            keyboard_reading: KeyboardReading from robot's keyboard sensor
        """
        # Clear previous states
        self.keys_pressed.clear()
        
        # Update with current pressed keys
        if keyboard_reading:
            for key in [KeyboardKey.O, KeyboardKey.P, KeyboardKey.C]:
                if key in keyboard_reading:
                    self.keys_pressed.add(key)
    
    def is_key_pressed(self, key: KeyboardKey) -> bool:
        """
        Check if a key is currently pressed
        
        Args:
            key: KeyboardKey to check
            
        Returns:
            bool: True if key is pressed
        """
        return key in self.keys_pressed
    
    def get_head_movement_command(self) -> str:
        """
        Get head movement command based on current key states
        
        Returns:
            str: 'left', 'right', 'none'
        """
        if self.is_key_pressed(KeyboardKey.O):
            return 'left'
        elif self.is_key_pressed(KeyboardKey.P):
            return 'right'
        else:
            return 'none'
    
    def is_calibration_complete_requested(self) -> bool:
        """
        Check if user has requested calibration completion
        
        Returns:
            bool: True if C key is pressed
        """
        return self.is_key_pressed(KeyboardKey.C)


class CalibrationState:
    """
    Manages the state of head calibration process
    """
    
    def __init__(self):
        self.phase = 'waiting_for_lidar'  # waiting_for_lidar, instructions_given, calibrating, complete
        self.lidar_data_count = 0
        self.min_lidar_points = 50
        self.instructions_spoken = False
        self.calibrated_position = 0.0
        self.start_time = None
        
    def update_lidar_data(self, obstacle_count: int):
        """
        Update with latest LiDAR data count
        
        Args:
            obstacle_count: Number of obstacles detected
        """
        self.lidar_data_count = obstacle_count
        
        if (self.phase == 'waiting_for_lidar' and 
            obstacle_count >= self.min_lidar_points):
            self.phase = 'ready_for_instructions'
    
    def mark_instructions_given(self):
        """Mark that instructions have been given to user"""
        self.instructions_spoken = True
        self.phase = 'calibrating'
        import time
        self.start_time = time.time()
    
    def complete_calibration(self, position: float):
        """
        Complete the calibration with the given position
        
        Args:
            position: Calibrated head position
        """
        self.calibrated_position = position
        self.phase = 'complete'
    
    def is_ready_for_instructions(self) -> bool:
        """Check if ready to give instructions"""
        return (self.phase == 'ready_for_instructions' and 
                not self.instructions_spoken)
    
    def is_calibrating(self) -> bool:
        """Check if in calibration mode"""
        return self.phase == 'calibrating'
    
    def is_complete(self) -> bool:
        """Check if calibration is complete"""
        return self.phase == 'complete'
    
    def get_status_message(self) -> str:
        """Get current status message for display"""
        if self.phase == 'waiting_for_lidar':
            return f"Waiting for LiDAR data... ({self.lidar_data_count}/{self.min_lidar_points})"
        elif self.phase == 'ready_for_instructions':
            return "LiDAR ready - giving instructions..."
        elif self.phase == 'calibrating':
            return "Use O (left) and P (right) to center head, press C when done"
        elif self.phase == 'complete':
            return "Calibration complete!"
        else:
            return "Unknown state"


def create_calibration_display_data(
    calibration_state: CalibrationState,
    current_head_position: float,
    lidar_obstacles: list
) -> Dict[str, Any]:
    """
    Create display data for calibration interface
    
    Args:
        calibration_state: Current calibration state
        current_head_position: Current head position
        lidar_obstacles: Current LiDAR obstacles
        
    Returns:
        dict: Display data for calibration interface
    """
    return {
        'status_message': calibration_state.get_status_message(),
        'phase': calibration_state.phase,
        'head_position': current_head_position,
        'lidar_count': len(lidar_obstacles) if lidar_obstacles else 0,
        'instructions': [
            "HEAD CALIBRATION MODE",
            "O = Move Left | P = Move Right | C = Save Center",
            "ESC = Exit to IDLE mode"
        ] if calibration_state.is_calibrating() else [
            "HEAD CALIBRATION STARTING",
            "Please wait for LiDAR initialization..."
        ],
        'show_lidar': len(lidar_obstacles) > 0 if lidar_obstacles else False,
        'obstacles': lidar_obstacles or []
    }


# Export main classes
__all__ = [
    'HeadCalibrationKeyHandler',
    'CalibrationState', 
    'create_calibration_display_data'
]