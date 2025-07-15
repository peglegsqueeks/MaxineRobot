"""
Pygame-only LiDAR plotting functionality
Replaces matplotlib-based plotting with pygame implementation
"""
import pygame
import math
import time
from typing import List, Optional, Tuple
from src.path_finding.Position import Position


class LidarPlot:
    """
    Pygame-based LiDAR plotting class
    Replaces matplotlib functionality with pygame for better real-time performance
    """
    
    def __init__(self, width=800, height=600, max_range=5000):
        """
        Initialize pygame-based LiDAR plot
        
        Args:
            width: Plot width in pixels
            height: Plot height in pixels
            max_range: Maximum LiDAR range in mm
        """
        self.width = width
        self.height = height
        self.max_range = max_range
        self.center_x = width // 2
        self.center_y = height // 2
        
        # Initialize pygame if not already done
        if not pygame.get_init():
            pygame.init()
        
        # Create surface (not a window)
        self.surface = pygame.Surface((width, height))
        
        # Colors
        self.colors = {
            'background': (0, 0, 0),
            'grid': (0, 120, 0),
            'obstacles': (255, 0, 0),
            'robot': (0, 255, 0),
            'target': (255, 255, 0),
            'path': (0, 255, 255),
            'text': (255, 255, 255)
        }
        
        # Font
        self.font = pygame.font.Font(None, 24)
        
        # Scale factor for display
        self.scale = min(width, height) // 10
        
    def clear(self):
        """Clear the plot"""
        self.surface.fill(self.colors['background'])
    
    def draw_grid(self):
        """Draw grid lines and distance rings"""
        # Draw distance rings
        for distance in range(1000, self.max_range, 1000):  # Every 1m
            radius = int(distance * self.scale / 1000)
            if radius < min(self.center_x, self.center_y):
                pygame.draw.circle(self.surface, self.colors['grid'], 
                                 (self.center_x, self.center_y), radius, 1)
        
        # Draw angle lines
        for angle in range(0, 360, 30):  # Every 30 degrees
            angle_rad = math.radians(angle)
            end_x = self.center_x + int(self.scale * 4 * math.cos(angle_rad))
            end_y = self.center_y + int(self.scale * 4 * math.sin(angle_rad))
            pygame.draw.line(self.surface, self.colors['grid'], 
                           (self.center_x, self.center_y), (end_x, end_y), 1)
    
    def draw_robot(self):
        """Draw robot at center"""
        pygame.draw.circle(self.surface, self.colors['robot'], 
                         (self.center_x, self.center_y), 10, 2)
        
        # Draw robot orientation (forward direction)
        end_x = self.center_x
        end_y = self.center_y - 20
        pygame.draw.line(self.surface, self.colors['robot'], 
                       (self.center_x, self.center_y), (end_x, end_y), 3)
    
    def draw_obstacles(self, obstacles: List[Position]):
        """Draw obstacles"""
        for obstacle in obstacles:
            if obstacle.distance > 0 and obstacle.distance < self.max_range:
                # Convert to screen coordinates
                x = self.center_x + int(obstacle.distance * math.sin(obstacle.angle) * self.scale / 1000)
                y = self.center_y - int(obstacle.distance * math.cos(obstacle.angle) * self.scale / 1000)
                
                # Draw obstacle point
                pygame.draw.circle(self.surface, self.colors['obstacles'], (x, y), 3)
    
    def draw_target(self, target: Position):
        """Draw target position"""
        if target and target.distance > 0:
            x = self.center_x + int(target.distance * math.sin(target.angle) * self.scale / 1000)
            y = self.center_y - int(target.distance * math.cos(target.angle) * self.scale / 1000)
            
            # Draw target as star
            pygame.draw.circle(self.surface, self.colors['target'], (x, y), 8)
            pygame.draw.circle(self.surface, self.colors['target'], (x, y), 8, 2)
    
    def draw_path(self, path: List[Position]):
        """Draw navigation path"""
        if not path or len(path) < 2:
            return
        
        points = []
        for pos in path:
            if pos.distance > 0:
                x = self.center_x + int(pos.distance * math.sin(pos.angle) * self.scale / 1000)
                y = self.center_y - int(pos.distance * math.cos(pos.angle) * self.scale / 1000)
                points.append((x, y))
        
        # Draw path lines
        if len(points) >= 2:
            pygame.draw.lines(self.surface, self.colors['path'], False, points, 2)
        
        # Draw waypoints
        for point in points:
            pygame.draw.circle(self.surface, self.colors['path'], point, 4)
    
    def add_text(self, text: str, position: Tuple[int, int]):
        """Add text to the plot"""
        text_surface = self.font.render(text, True, self.colors['text'])
        self.surface.blit(text_surface, position)
    
    def update(self, obstacles: List[Position] = None, target: Position = None, 
               path: List[Position] = None):
        """
        Update the plot with new data
        
        Args:
            obstacles: List of obstacle positions
            target: Target position
            path: Navigation path
        """
        # Clear and redraw
        self.clear()
        self.draw_grid()
        self.draw_robot()
        
        if obstacles:
            self.draw_obstacles(obstacles)
        
        if target:
            self.draw_target(target)
        
        if path:
            self.draw_path(path)
        
        # Add status text
        if obstacles:
            self.add_text(f"Obstacles: {len(obstacles)}", (10, 10))
        
        if target:
            self.add_text(f"Target: {target.distance:.0f}mm, {math.degrees(target.angle):.1f}°", (10, 30))
        
        if path:
            self.add_text(f"Path: {len(path)} waypoints", (10, 50))
    
    def get_surface(self) -> pygame.Surface:
        """Get the pygame surface"""
        return self.surface
    
    def save_image(self, filename: str):
        """Save plot as image"""
        pygame.image.save(self.surface, filename)
    
    def close(self):
        """Close/cleanup the plot"""
        # For pygame surface, just clear it
        self.surface.fill((0, 0, 0))


# Compatibility functions for any code that might import these
def create_lidar_plot(width=800, height=600, max_range=5000):
    """Create a new LiDAR plot"""
    return LidarPlot(width, height, max_range)


def plot_lidar_data(obstacles: List[Position], target: Position = None, 
                   path: List[Position] = None, title: str = "LiDAR Data"):
    """
    Plot LiDAR data (compatibility function)
    Returns a pygame surface instead of showing matplotlib plot
    """
    plot = LidarPlot()
    plot.update(obstacles, target, path)
    return plot.get_surface()


# Legacy matplotlib-style interface for backward compatibility
class pyplot:
    """Dummy pyplot interface that does nothing"""
    
    @staticmethod
    def figure(*args, **kwargs):
        pass
    
    @staticmethod
    def subplot(*args, **kwargs):
        pass
    
    @staticmethod
    def plot(*args, **kwargs):
        pass
    
    @staticmethod
    def scatter(*args, **kwargs):
        pass
    
    @staticmethod
    def xlim(*args, **kwargs):
        pass
    
    @staticmethod
    def ylim(*args, **kwargs):
        pass
    
    @staticmethod
    def xlabel(*args, **kwargs):
        pass
    
    @staticmethod
    def ylabel(*args, **kwargs):
        pass
    
    @staticmethod
    def title(*args, **kwargs):
        pass
    
    @staticmethod
    def show(*args, **kwargs):
        pass
    
    @staticmethod
    def close(*args, **kwargs):
        pass
    
    @staticmethod
    def clf(*args, **kwargs):
        pass
    
    @staticmethod
    def pause(*args, **kwargs):
        time.sleep(0.01)  # Small delay


# Create pyplot instance for backward compatibility
plt = pyplot()

# Export main classes and functions
__all__ = ['LidarPlot', 'create_lidar_plot', 'plot_lidar_data', 'plt', 'pyplot']