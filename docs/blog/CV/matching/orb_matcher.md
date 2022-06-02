---
title: opencv feature matching
description: using ORB and Matcher 
date: "2022-05-14"
banner: ../images/logo.jpeg
tags:
    - orb
    - matching
    - opencv
---

### Hamming Distance
Hamming Distance measures the similarity between two strings of the same length.
Hamming distance is the number of bit positions in which the two bits are different.

![](images/hamming_distance.png)

---

### BFMatcher
Brute-Force matching take each feature from image `query` and compares it to all the other/train image 


### Matcher result
[match method](https://docs.opencv.org/4.x/db/d39/classcv_1_1DescriptorMatcher.html)

[DMatch](https://docs.opencv.org/4.x/d4/de0/classcv_1_1DMatch.html#details)


- distance
- imgIdx: train image index
- queryIdx: query descriptor index
- trainIdx: train descriptor index

### Basic Demo
```python
import numpy as np
import cv2
  
     
# Read the query image as query_img
# and train image This query image
# is what you need to find in train image
# Save it in the same directory
# with the name image.jpg 
query_img = cv2.imread('query.jpg')
train_img = cv2.imread('train.jpg')
  
# Convert it to grayscale
query_img_bw = cv2.cvtColor(query_img,cv2.COLOR_BGR2GRAY)
train_img_bw = cv2.cvtColor(train_img, cv2.COLOR_BGR2GRAY)
  
# Initialize the ORB detector algorithm
orb = cv2.ORB_create()
  
# Now detect the keypoints and compute
# the descriptors for the query image
# and train image
queryKeypoints, queryDescriptors = orb.detectAndCompute(query_img_bw, mask=None)
trainKeypoints, trainDescriptors = orb.detectAndCompute(train_img_bw, mask=None)
 
# Initialize the Matcher for matching
# the keypoints and then match the
# keypoints
matcher = cv2.BFMatcher()
matches = matcher.match(queryDescriptors,trainDescriptors)
  
# draw the matches to the final image
# containing both the images the drawMatches()
# function takes both images and keypoints
# and outputs the matched query image with
# its train image
final_img = cv2.drawMatches(query_img, queryKeypoints,
train_img, trainKeypoints, matches[:20],None)
  
final_img = cv2.resize(final_img, (1000,650))
 
# Show the final image
cv2.imshow("Matches", final_img)
cv2.waitKey(3000)
```
---

# Reference
- [OpenCV-python-tests](https://github.com/jagracar/OpenCV-python-tests/find/master)
- [Different types of distance used in Machine Learning.](https://medium.com/@anjaliksahar/different-types-of-distance-used-in-machine-learning-6498c77b4423)
- [comparing_images](https://vzat.github.io/comparing_images/week5.html)