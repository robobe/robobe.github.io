import numpy as np
import cv2
from os import path

# https://www.bogotobogo.com/python/OpenCV_Python/images/mean_shift_tracking/
url = path.join(path.dirname(__file__), "slow_traffic_small.mp4")
cap = cv2.VideoCapture(url)
font = cv2.FONT_HERSHEY_SIMPLEX
# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
color = np.random.randint(0, 255, (100, 3))
ret, first_frame = cap.read()
#roi variable will save xy coordinate of left top corner and bottom right corner point of bounding box
roi=cv2.selectROI("select", first_frame)
cv2.destroyWindow("select")
old_gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)

mask = np.zeros_like(old_gray)
mask[int(roi[1]):int(roi[1]+roi[3]), int(roi[0]):int(roi[0]+roi[2])] = 255
p0 = cv2.goodFeaturesToTrack(old_gray, mask = mask, **feature_params)

mask = np.zeros_like(first_frame)
 
while True:
    ret, frame = cap.read()
    if not ret:
        print("no frame grabbed")
        break
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, gray_frame, p0, None, **lk_params)
    
    # Select good points
    if p1 is not None:
        good_new = p1[st==1]
        good_old = p0[st==1]

        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            # mask not renew each frame, lines draw from the first image 
            mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
            frame = cv2.circle(frame, (int(a), int(b)), 5, color[i].tolist(), -1)
    img = cv2.add(frame, mask)
    cv2.putText(img,f'good_new: {good_new.size/2}',(10,60), font, 0.7,(0,255,0),1,cv2.LINE_AA)
    cv2.putText(img,f'good_old: {good_old.size/2}',(10,80), font, 0.7,(0,255,0),1,cv2.LINE_AA)
    cv2.imshow("lk", img)
    k = cv2.waitKey(30) & 0xFF
    if k == 27:
        break

    # Now update the previous frame and previous points
    old_gray = gray_frame.copy()
    p0 = good_new.reshape(-1, 1, 2)

cap.release()
cv2.destroyAllWindows()
