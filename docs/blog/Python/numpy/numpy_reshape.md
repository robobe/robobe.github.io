---
title: numpy reshape
description: hello numpy
date: "2022-05-11"
banner: ../numpy.png
tags:
    - python
    - numpy
---
The new shape should be compatible with the original shape'

!!! Tips title
    Set one of the reshape parameters as `-1`  
    Allow numpy to figure unknown dimension
    keep the above criteria  

    ```python
    s = np.array([[1, 2, 3, 4],
         [5, 6, 7, 8],
         [9, 10, 11, 12]])
    s.shape
    (3,4)

    s1 = np.reshape(s, -1)
    s1.shape
    (12,)

    s2 = s.reshape(1, -1)
    s2.shape
    (1, 12)

    s3 = s.reshape(-1, 1, 2)
    (6, 1, 2)
    s3
    array([
        [[ 1,  2]],
        [[ 3,  4]],
        [[ 5,  6]],
        [[ 7,  8]],
        [[ 9, 10]],
        [[11, 12]]])

    ```
---

# Reference
- [Numpy Tutorial – Your first numpy guide to build python coding foundations](https://www.machinelearningplus.com/python/numpy-tutorial-part1-array-python-examples/)
- [Python NumPy For Your Grandma](https://www.gormanalysis.com/blog/python-numpy-for-your-grandma-1-1-introduction/)