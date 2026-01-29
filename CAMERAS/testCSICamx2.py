import cv2

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

cap0 = cv2.VideoCapture(gstreamer_pipeline(0), cv2.CAP_GSTREAMER)
cap1 = cv2.VideoCapture(gstreamer_pipeline(1), cv2.CAP_GSTREAMER)

if not cap0.isOpened():
    print("❌ Failed to open camera 0")
    exit(1)

if not cap1.isOpened():
    print("❌ Failed to open camera 1")
    exit(1)

print("✅ Both cameras opened via Argus")

while True:
    ret0, frame0 = cap0.read()
    ret1, frame1 = cap1.read()

    if not ret0 or not ret1:
        print("❌ Failed to read frame")
        break

    cv2.imshow("Camera 0", frame0)
    cv2.imshow("Camera 1", frame1)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap0.release()
cap1.release()
cv2.destroyAllWindows()
