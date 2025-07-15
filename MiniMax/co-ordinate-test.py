#!/usr/bin/env python3
"""
Quick test script to verify head angle coordinate transformation
Run this to check if the coordinate math is correct
"""

import math

def test_coordinate_transform():
    """Test the coordinate transformation math"""
    
    print("🔧 Testing Head Angle Coordinate Transformation")
    print("=" * 60)
    
    # Test scenarios
    test_cases = [
        {"head_angle": 0, "description": "Head centered"},
        {"head_angle": 30, "description": "Head turned 30° right"},
        {"head_angle": -30, "description": "Head turned 30° left"},
        {"head_angle": 45, "description": "Head turned 45° right"},
    ]
    
    # Test point: person directly in front of camera (0, 1000mm)
    x_camera = 0      # Directly in front of camera
    z_camera = 1000   # 1 meter away
    
    for test in test_cases:
        head_angle_degrees = test["head_angle"]
        description = test["description"]
        
        print(f"\n📍 {description} ({head_angle_degrees}°)")
        print("-" * 40)
        
        # Apply coordinate transformation
        head_angle_rad = math.radians(head_angle_degrees)
        
        # Transform camera coordinates to robot body coordinates  
        x_body = x_camera * math.cos(head_angle_rad) - z_camera * math.sin(head_angle_rad)
        z_body = x_camera * math.sin(head_angle_rad) + z_camera * math.cos(head_angle_rad)
        
        # Calculate angles
        body_angle_rad = math.atan2(x_body, z_body)
        body_angle_deg = math.degrees(body_angle_rad)
        
        distance = math.sqrt(x_body*x_body + z_body*z_body)
        
        print(f"Camera coords: ({x_camera}, {z_camera})")
        print(f"Body coords:   ({x_body:.0f}, {z_body:.0f})")
        print(f"Body angle:    {body_angle_deg:.1f}°")
        print(f"Distance:      {distance:.0f}mm")
        
        # Verify logic
        if head_angle_degrees == 0:
            expected_body_angle = 0
        else:
            expected_body_angle = -head_angle_degrees  # Person appears opposite to head turn
            
        print(f"Expected:      {expected_body_angle:.1f}°")
        
        error = abs(body_angle_deg - expected_body_angle)
        if error < 0.1:
            print("✅ CORRECT")
        else:
            print(f"❌ ERROR: {error:.1f}° difference")
    
    print("\n" + "=" * 60)
    print("💡 Expected behavior:")
    print("   - Head center (0°): Person at 0° relative to body")
    print("   - Head right (+30°): Person at -30° relative to body")  
    print("   - Head left (-30°): Person at +30° relative to body")
    print("=" * 60)

if __name__ == "__main__":
    test_coordinate_transform()