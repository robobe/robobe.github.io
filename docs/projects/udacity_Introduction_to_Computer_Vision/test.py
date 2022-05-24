import cv2

image = cv2.imread("/home/user/Pictures/test.png")
image[:,:,0] = 0
image[:,:,2] = 0
cv2.imshow("title", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
