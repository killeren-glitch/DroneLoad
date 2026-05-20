import cv2

# Minimal local test
gst_test = "videotestsrc num-buffers=60 ! videoconvert ! appsink"
cap = cv2.VideoWriter(gst_test, cv2.CAP_GSTREAMER, 0, 30, (640, 480))

print(f"Is opened: {cap.isOpened()}")