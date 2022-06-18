---
title: Image Transforms and Rotation
description: Image Transforms and Rotation
date: "2022-06-09"
banner: ../images/camera.png
tags:
    - transformation
    - computer vision
---

# Translation

![](images/translation.png)

## opencv demo

```python
--8<-- "examples/open_cv/computer_vision/transform.py"
```

![](images/lana_translation.png)

---

# Rotation

![](images/rotation.png)

![](images/homeogenous_matrix.png)


```python title="rotate" 
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

```

![](images/lana_rotate.png)

---

# Scale

![](images/scale.png)




---

# Reference
- [Pratik jain computer vision](https://www.youtube.com/watch?v=vqdshHpkvkg&list=PLFXza2AmUJa-RJZJj3b5Wm76I9k_QthWC)
- [How to resize, translate, flip and rotate an image with OpenCV](https://datahacker.rs/003-how-to-resize-translate-flip-and-rotate-an-image-with-opencv/)
- [understanding-geometric-transformation](https://theailearner.com/2020/11/01/understanding-geometric-transformation-translation-using-opencv-python/)