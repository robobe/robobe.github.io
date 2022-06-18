import cv2
import numpy as np

image = cv2.imread('/home/user/projects/blog/examples/open_cv/homography/images/lana.png')

height, width = image.shape[:2]
center = (width / 2, height / 2)
angle = 45
scale = 1
M = cv2.getRotationMatrix2D(center, angle, scale)
img_translation = cv2.warpAffine(image, M, (width, height))

cv2.imshow("src", image)
cv2.imshow("dst", img_translation)

cv2.waitKey(0)
