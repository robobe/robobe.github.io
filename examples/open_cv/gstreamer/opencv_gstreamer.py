import cv2

cap = cv2.VideoCapture("videotestsrc ! video/x-raw,width=640,height=480,framerate=10/1 ! videoconvert ! timeoverlay ! appsink")
out_pipe = "appsrc ! video/x-raw,width=640,height=480,framerate=10/1 ! videoconvert ! timeoverlay xpad=100 ypad=100 ! autovideosink sync=false"
out = cv2.VideoWriter(out_pipe, 0, 10.0, (640,480))

while True:
    ret, frame = cap.read()
    cv2.imshow("cv", frame)
    out.write(frame)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
cap.release()
out.release()
cv2.destroyAllWindows()