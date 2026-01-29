import cv2

# =========================
# GStreamer pipeline for CSI cameras
# =========================
def gstreamer_pipeline(sensor_id, width=1280, height=720, fps=30):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, framerate={fps}/1 ! "
        f"nvvidconv ! "
        f"video/x-raw, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! "
        f"appsink drop=true sync=false"
    )

# =========================
# Open CSI cameras (Argus)
# =========================
cap_csi_0 = cv2.VideoCapture(
    gstreamer_pipeline(sensor_id=0), cv2.CAP_GSTREAMER
)
cap_csi_1 = cv2.VideoCapture(
    gstreamer_pipeline(sensor_id=1), cv2.CAP_GSTREAMER
)

# =========================
# Open USB camera (V4L2)
# =========================
cap_usb = cv2.VideoCapture(2, cv2.CAP_V4L2)

# 建议给 USB 降点规格，防止抢带宽
cap_usb.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap_usb.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
cap_usb.set(cv2.CAP_PROP_FPS, 30)

# =========================
# Check all cameras
# =========================
if not cap_csi_0.isOpened():
    print("❌ Failed to open CSI camera 0")
    exit(1)

if not cap_csi_1.isOpened():
    print("❌ Failed to open CSI camera 1")
    exit(1)

if not cap_usb.isOpened():
    print("❌ Failed to open USB camera")
    exit(1)

print("✅ All 3 cameras opened successfully")

# =========================
# Main loop
# =========================
while True:
    ret0, frame0 = cap_csi_0.read()
    ret1, frame1 = cap_csi_1.read()
    retu, frameu = cap_usb.read()

    if not ret0 or not ret1 or not retu:
        print("❌ Failed to read from one of the cameras")
        break

    cv2.imshow("CSI Camera 0", frame0)
    cv2.imshow("CSI Camera 1", frame1)
    cv2.imshow("USB Camera", frameu)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# =========================
# Cleanup
# =========================
cap_csi_0.release()
cap_csi_1.release()
cap_usb.release()
cv2.destroyAllWindows()
