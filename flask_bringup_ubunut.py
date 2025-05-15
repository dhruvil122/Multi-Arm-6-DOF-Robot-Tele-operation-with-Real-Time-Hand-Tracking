import cv2

cap = cv2.VideoCapture("http://192.168.64.1:5000/video_feed")

while True:
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)
    if ret:
        cv2.imshow("Mac Camera Stream", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
