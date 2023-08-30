---
tags:
    - matrix
    - graphics
    - rotation
    - 2d
---
# 2D Rotation

$
\begin{bmatrix}
x & y & 1
\end{bmatrix}
\rightarrow
\begin{bmatrix}
{x}' & {y}' & 1
\end{bmatrix}
$

$
\begin{bmatrix}
x & y & 1
\end{bmatrix}
\begin{bmatrix}
cos\Theta & sin\Theta  & 0 \\
-sin\Theta & cos\Theta  & 0 \\
0 & 0 & 1
\end{bmatrix}
$

### Demo 
90 degree rotation p = (1, 0)

$
sin90 = 1 \\
cos90 = 0
$



$
\begin{bmatrix}
1 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
0 & 1  & 0 \\
-1 & 0  & 0 \\
0 & 0 & 1
\end{bmatrix}
= \\
\begin{bmatrix}
1\cdot0 + 0\cdot-1+1\cdot0
 & 
1\cdot1 + 0\cdot0+1\cdot0
 & 
1\cdot0 + 0\cdot0+1\cdot1
\end{bmatrix}
= \\
\begin{bmatrix}
0 & 1 & 1
\end{bmatrix}
$
