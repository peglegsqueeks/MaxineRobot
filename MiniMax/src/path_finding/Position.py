import math

class Position:
    """
    Represents a position in polar coordinates (angle, distance)
    Used for lidar obstacle detection and robot navigation
    """
    
    def __init__(self, angle: float = 0.0, distance: float = 0.0):
        """
        Initialize position with angle and distance
        
        Args:
            angle: Angle in radians (0 = forward, positive = right)
            distance: Distance in millimeters
        """
        self.angle = angle
        self.distance = distance
    
    def to_cartesian(self):
        """
        Convert polar coordinates to cartesian coordinates
        
        Returns:
            tuple: (x, y) coordinates in millimeters
            x = right/left, y = forward/backward
        """
        x = self.distance * math.sin(self.angle)  # Right/left
        y = self.distance * math.cos(self.angle)  # Forward/backward
        return (x, y)
    
    def from_cartesian(self, x: float, y: float):
        """
        Create position from cartesian coordinates
        
        Args:
            x: Right/left coordinate in millimeters
            y: Forward/backward coordinate in millimeters
        """
        self.distance = math.sqrt(x*x + y*y)
        self.angle = math.atan2(x, y)
        return self
    
    def __str__(self):
        """String representation of the position"""
        angle_deg = math.degrees(self.angle)
        return f"Position(angle={angle_deg:.1f}°, distance={self.distance:.1f}mm)"
    
    def __repr__(self):
        """Detailed string representation"""
        return self.__str__()