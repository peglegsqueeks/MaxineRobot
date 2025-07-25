import depthai as dai
import cv2

# Create pipeline
pipeline = dai.Pipeline()

# === Create mono cameras ===
left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)

left.setBoardSocket(dai.CameraBoardSocket.LEFT)
right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# ✅ Use full 800p resolution for full FOV
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)

# === Create StereoDepth ===
stereo = pipeline.create(dai.node.StereoDepth)

# ✅ Use HIGH_ACCURACY to ensure IR dot projector is on
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

# ✅ Use full FOV: do not enable extended disparity, do not crop.
# Extended disparity crops the FOV horizontally — so we keep it disabled.
stereo.setExtendedDisparity(False)

# Optionally enable LR-check and subpixel for better depth, no FOV impact.
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)

# Link mono cams to stereo depth
left.out.link(stereo.left)
right.out.link(stereo.right)

# === Output ===
xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutDepth.setStreamName("depth")
stereo.depth.link(xoutDepth.input)

# === Run pipeline ===
with dai.Device(pipeline) as device:
    print("Running OAK-D Pro W test at 800p, FULL FOV, laser dot projector ON (auto)")

    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    while True:
        inDepth = depthQueue.get()
        depthFrame = inDepth.getFrame()

        # Normalize for visualization
        depthFrame = cv2.normalize(depthFrame, None, 0, 255, cv2.NORM_MINMAX)
        depthFrame = cv2.convertScaleAbs(depthFrame)

        cv2.imshow("Depth", depthFrame)

        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
