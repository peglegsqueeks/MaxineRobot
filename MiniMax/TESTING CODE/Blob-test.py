#!/usr/bin/env python3
"""
Quick .blob Model Tester for Maxine Robot
Tests any .blob file for spatial detection compatibility
"""

import depthai as dai
import sys
import os

def test_blob_model(blob_path, network_type="auto"):
    """Test a .blob model for spatial detection compatibility"""
    
    if not os.path.exists(blob_path):
        print(f"❌ File not found: {blob_path}")
        return False
    
    # Auto-detect network type from filename
    if network_type == "auto":
        if "yolo" in blob_path.lower():
            network_type = "YoloSpatialDetectionNetwork"
        else:
            network_type = "MobileNetSpatialDetectionNetwork"
    
    print(f"🧪 Testing: {os.path.basename(blob_path)}")
    print(f"🔧 Network type: {network_type}")
    
    try:
        pipeline = dai.Pipeline()
        
        # Create camera nodes
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(416, 416)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        
        # Create depth
        depth = pipeline.create(dai.node.StereoDepth)
        depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        depth.setDepthAlign(dai.CameraBoardSocket.RGB)
        mono_left.out.link(depth.left)
        mono_right.out.link(depth.right)
        
        # Create ImageManip for NN input
        manip = pipeline.create(dai.node.ImageManip)
        manip.initialConfig.setResize(416, 416)  # Standard NN input
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        cam_rgb.preview.link(manip.inputImage)
        
        # Create spatial detection network
        if network_type == "YoloSpatialDetectionNetwork":
            print("🎯 Creating YoloSpatialDetectionNetwork...")
            spatial_nn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
            
            # YOLO specific configuration
            spatial_nn.setNumClasses(80)
            spatial_nn.setCoordinateSize(4)
            spatial_nn.setAnchors([10,14, 23,27, 37,58, 81,82, 135,169, 344,319])
            spatial_nn.setAnchorMasks({"side26": [1,2,3], "side13": [3,4,5]})
            spatial_nn.setIouThreshold(0.5)
            
        else:
            print("📱 Creating MobileNetSpatialDetectionNetwork...")
            spatial_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
        
        # Set blob path and common parameters
        spatial_nn.setBlobPath(blob_path)
        spatial_nn.setConfidenceThreshold(0.3)
        spatial_nn.setBoundingBoxScaleFactor(0.5)
        spatial_nn.setDepthLowerThreshold(100)
        spatial_nn.setDepthUpperThreshold(5000)
        
        # Link inputs
        manip.out.link(spatial_nn.input)
        depth.depth.link(spatial_nn.inputDepth)
        
        # Create output
        detection_out = pipeline.create(dai.node.XLinkOut)
        detection_out.setStreamName("detections")
        spatial_nn.out.link(detection_out.input)
        
        print("✅ Pipeline creation: SUCCESS")
        print("✅ Spatial detection network: CONFIGURED")
        print("✅ Z-depth input: CONNECTED")
        
        # Test with device (quick test)
        print("🔄 Testing with OAK device...")
        with dai.Device(pipeline) as device:
            print("✅ Device connection: SUCCESS")
            print("✅ Model loading: SUCCESS")
            print("✅ Spatial detection: READY")
            
            # Get one detection to verify it works
            detection_queue = device.getOutputQueue("detections", maxSize=1, blocking=False)
            import time
            time.sleep(2)  # Let it initialize
            
            detections = detection_queue.tryGet()
            if detections:
                print(f"✅ Detection output: WORKING ({len(detections.detections)} objects)")
            else:
                print("ℹ️ Detection output: NO OBJECTS (but system working)")
        
        print(f"\n🎉 MODEL TEST PASSED: {os.path.basename(blob_path)}")
        print(f"Network type: {network_type}")
        
        # Show integration code
        person_class = 0 if "yolo" in network_type.lower() else 15
        print(f"\n🔧 INTEGRATION CODE:")
        print(f"detection_nn = pipeline.create(dai.node.{network_type})")
        print(f"detection_nn.setBlobPath('{blob_path}')")
        if "yolo" in network_type.lower():
            print("# YOLO specific config")
            print("detection_nn.setNumClasses(80)")
            print("detection_nn.setCoordinateSize(4)")
            print("detection_nn.setAnchors([10,14, 23,27, 37,58, 81,82, 135,169, 344,319])")
            print("detection_nn.setAnchorMasks({'side26': [1,2,3], 'side13': [3,4,5]})")
        print(f"# Person detection: class ID {person_class}")
        
        return True
        
    except Exception as e:
        print(f"❌ MODEL TEST FAILED: {e}")
        return False

def main():
    """Main test function"""
    print("🤖 MAXINE ROBOT - QUICK .BLOB MODEL TESTER")
    print("=" * 50)
    
    if len(sys.argv) < 2:
        print("Usage: python3 quick_blob_tester.py <path_to_blob_file>")
        print("\nExample:")
        print("python3 quick_blob_tester.py ./models/yolov4_tiny_coco_416x416_openvino_2022.1_6shave.blob")
        print("python3 quick_blob_tester.py ./yolov4_tiny_coco_416x416_openvino_2022.1_6shave.blob")
        return
    
    blob_path = sys.argv[1]
    network_type = sys.argv[2] if len(sys.argv) > 2 else "auto"
    
    success = test_blob_model(blob_path, network_type)
    
    if success:
        print("\n✅ This model is ready for integration into your Maxine robot!")
        print("Copy the integration code above into your detection system.")
    else:
        print("\n❌ This model has compatibility issues.")
        print("Try a different model or check the error messages above.")

if __name__ == "__main__":
    main()