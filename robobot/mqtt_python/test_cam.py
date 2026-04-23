import cv2
cap = cv2.VideoCapture("http://127.0.0.1:7123/stream/main")
ret, frame = cap.read()
if ret:
    cv2.imwrite("test_frame.jpg", frame)
    print("Frame saved: shape", frame.shape)
else:
    print("Failed to grab frame.")
cap.release()
