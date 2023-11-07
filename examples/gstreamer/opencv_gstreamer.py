import cv2


FPS = 10
SIZE = (640, 480)
# FourCC is a 4-byte code used to specify the video codec
FOURCC = 0 # uncompressed

cap = cv2.VideoCapture(f"videotestsrc ! video/x-raw,width=640,height=480,framerate={FPS}/1 ! videoconvert ! timeoverlay ! appsink")
out_pipe = f"appsrc ! video/x-raw,width=640,height=480,framerate={FPS}/1 ! videoconvert ! timeoverlay xpad=100 ypad=100 ! autovideosink sync=false"
out = cv2.VideoWriter(out_pipe, FOURCC, FPS, SIZE)

while True:
    ret, frame = cap.read()
    cv2.imshow("cv", frame)
    out.write(frame)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
cap.release()
out.release()
cv2.destroyAllWindows()