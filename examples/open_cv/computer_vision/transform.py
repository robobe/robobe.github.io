import cv2
import numpy as np

image = cv2.imread('/home/user/projects/blog/examples/open_cv/homography/images/lana.png')

height, width = image.shape[:2]
tx = 100
ty = 100
T = np.float32([[1, 0, tx], [0, 1, ty]])

img_translation = cv2.warpAffine(image, T, (width, height))

cv2.imshow("src", image)
cv2.imshow("dst", img_translation)

cv2.warpAffine(image, T, (width, height))
cv2.waitKey(0)
