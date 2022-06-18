import cv2
import numpy as np

img = cv2.imread('/home/user/projects/blog/examples/open_cv/homography/images/lana.png')

rows,cols,_ = img.shape

# Create the transformation matrix
M = np.float32([[1,0,100],[0,1,50]])
M = np.float32([[0.5, 0],[0, 0.5]])
# get the coordinates in the form of (0,0),(0,1)...
# the shape is (2, rows*cols)
orig_coord = np.indices((cols, rows)).reshape(2,-1)
# stack the rows of 1 to form [x,y,1]
orig_coord_f = np.vstack((orig_coord, np.ones(rows*cols)))
transform_coord = np.dot(M, orig_coord)
# Change into int type
transform_coord = transform_coord.astype(np.int32)
# Keep only the coordinates that fall within the image boundary.
indices = np.all((transform_coord[1]<rows, transform_coord[0]<cols, transform_coord[1]>=0, transform_coord[0]>=0), axis=0)
# Create a zeros image and project the points
img1 = np.zeros_like(img)
img1[transform_coord[1][indices], transform_coord[0][indices]] = img[orig_coord[1][indices], orig_coord[0][indices]]
# Display the image
out = cv2.hconcat([img,img1])
cv2.imshow('a2',out)
cv2.waitKey(0)