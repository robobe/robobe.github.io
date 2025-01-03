---
tags:
    - ros2
    - euler
    - quaternion
    - tf
---

# tf_transformations

A library in ROS (or Python) provides a collection of utility functions for handling transformations, rotations, translations, and their representations in quaternions and Euler angles.




## euler
- euler_matrix
- translation_matrix: Creates a 4×4 transformation matrix representing a translation
- rotation_matrix: Creates a 4×4 transformation matrix representing a rotation about an axis


```python
from tf_transformations import translation_matrix
matrix = translation_matrix([1, 2, 3])
print(matrix)
#
[[1. 0. 0. 1.]
 [0. 1. 0. 2.]
 [0. 0. 1. 3.]
 [0. 0. 0. 1.]]
```


#### Demo: Rotate on z-axis
```python
import math
import numpy as np
from tf_transformations import rotation_matrix

matrix = rotation_matrix(math.pi/2, [0, 0, 1])
matrix = np.round(matrix, decimals=3)
print(matrix)

# 
[[ 0. -1.  0.  0.]
 [ 1.  0.  0.  0.]
 [ 0.  0.  1.  0.]
 [ 0.  0.  0.  1.]]

```

---

## homogeneous matrix
A homogeneous matrix (or homogeneous transformation matrix) is a matrix used to represent transformations (such as rotation, translation, scaling, and affine transformations) in a more compact and efficient form


```python
rotate = rotation_matrix(math.pi/2, [0, 0, 1])
rotate = np.round(rotate, decimals=3)
print(rotate)

translate = translation_matrix([1, 2, 3])
print(translate)

matrix = np.dot(translate, rotate)
print(matrix)

#
[[ 0. -1.  0.  1.]
 [ 1.  0.  0.  2.]
 [ 0.  0.  1.  3.]
 [ 0.  0.  0.  1.]]
```

---

### Quaternion

- euler_from_quaternion
- quaternion_from_euler
- quaternion_multiply


```python
quat = quaternion_from_euler(0, 0, pi/2)
roll, pitch, yaw = euler_from_quaternion(quat)
print(roll, pitch, yaw)
```


#### quaternion vector multiplication
Quaternion multiplication is a mathematical operation used in 3D rotations to combine rotations or apply a rotation to a vector. When applying a quaternion to a vector, the vector is first converted to a quaternion, and then quaternion multiplication is used.

$
v_{rotate} = q\cdot v\cdot q^{-1}
$

- q: is the quaternion representing the rotation.
- $q^{-1}$:  is the conjugate of the quaternion (for unit quaternions).
- v: is the vector, treated as a quaternion with w=0: [x,y,z,0].
- ⋅ : denotes quaternion multiplication.

##### demo

rotate vector by 90 degree in the z axis
input_vector = [1,0,0]

```python
from tf_transformations import (
    quaternion_multiply,
    quaternion_from_euler,
    quaternion_conjugate
)

import math
import numpy as np

#input vector
vector = np.array([1, 0, 0])  # Example vector
# create rotation
quat = quaternion_from_euler(0, 0, math.pi/2)
vector_as_quat = [vector[0], vector[1], vector[2], 0]
# conjugate quaternion
q_conjugate = quaternion_conjugate(quat)
#multiple
q = quaternion_multiply(quaternion_multiply(quat, vector_as_quat), q_conjugate)
# extract vector from result
v = np.array(q[:3])
# round for batter view
v= np.round(v, 1)
print(v)
```


---

## ROS2 Coordinate system

[Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)


| frame  |   | ROS  |
|---|---|---|
| Body  | FRD (X Forward, Y Right, Z Down)  | FLU (X Forward, Y Left, Z Up)  |
| World  | FRD or NED (X North, Y East, Z Down)  | FLU or ENU (X East, Y North, Z Up)  |


#### ENU to NED
- rotate PI/2 around Z 
- rotate PI around X

```python
from tf_transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    rotation_matrix,
    translation_matrix
)

from math import pi
import numpy as np


y = rotation_matrix(pi/2,[0,0,1])
r = rotation_matrix(pi,[1,0,0])

enu_2_ned = y@r

enu = np.array([1.0 , 0.0, 0.0, 1.0])

ned = enu_2_ned@enu.T

print(f"enu: {np.round(enu, 1)}")
print(f"enu: {np.round(ned, 1)}")
```

#### NED to ENU

Same matrix like enu->ned

demo code convert the rotation matrix to quaternion for vector multiple

```python
from tf_transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    rotation_matrix,
    quaternion_from_matrix,
    quaternion_conjugate,
    quaternion_multiply
    
)

from math import pi
import numpy as np


y = rotation_matrix(pi/2,[0,0,1])
r = rotation_matrix(pi,[1,0,0])

ned_2_enu = y@r
q = quaternion_from_matrix(ned_2_enu)
qc = quaternion_conjugate(q)

ned = quaternion_multiply(quaternion_multiply(q,enu), qc)[:3]

print(f"ned: {np.round(ned, 1)}")
```


#### FRD to FLU / FLU to FRD

rotate PI around x axis

