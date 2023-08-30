
---
tags:
    - matrix
    - graphics
    - 2d
---
# 2D

## Point in 2d

$
p = x\vec{u} + y\vec{v} + o
$

$
p = \begin{bmatrix}
x & y & 1
\end{bmatrix}\begin{bmatrix}
\vec{u} \\
\vec{u} \\ 
o
\end{bmatrix}
$

---

## Translation

$
p = \begin{bmatrix}
x & y & 1
\end{bmatrix}
\rightarrow
\begin{bmatrix}
x+a & y+b & 1
\end{bmatrix}
$

---

$
\begin{bmatrix}
    x & y & 1
\end{bmatrix}
\begin{bmatrix}
    1 & 0 & 0 \\
    0 & 1 & 0 \\
    a & b & 1
\end{bmatrix} = 
\begin{bmatrix}
    x+a & y+b & 1
\end{bmatrix}
$



---

## Scaleing

$
\begin{bmatrix}
    x & y & 1
\end{bmatrix}
\rightarrow
\begin{bmatrix}
    ax & by & 1
\end{bmatrix}
$

$
\begin{bmatrix}
    x & y & 1
\end{bmatrix}
\begin{bmatrix}
a & 0 & 0 \\
0 & b & 0 \\
0 & 0 & 1 \\
\end{bmatrix}=
\begin{bmatrix}
ax & by & 1
\end{bmatrix}
$


## TST
- Translate to origin
- Scale
- Translate back

$
\begin{bmatrix}
    1 & 0 & 0 \\
    0 & 1 & 0 \\
    -a & -b & 1
\end{bmatrix}
\begin{bmatrix}
    Sx & 0 & 0 \\
    0 & Sy & 0 \\
    0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
    1 & 0 & 0 \\
    0 & 1 & 0 \\
    -a & -b & 1
\end{bmatrix}
$