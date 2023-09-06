---
tags:
    - cupy
    - numpy
    - gpu
---

# CuPy
CuPy is a NumPy/SciPy-compatible array library for GPU-accelerated computing with Python. CuPy acts as a drop-in replacement to run existing NumPy/SciPy code on NVIDIA CUDA 

---

## Install
- nvidia driver 
- cuda toolkit
- cupy

### NVidia driver

![](images/nvidia-smi.png)

### CUDA Toolkit

```bash
sudo apt install nvidia-cuda-toolkit nvidia-cuda-toolkit-gcc
```

```bash title="check cuda version" linenums="1" hl_lines="7"
nvcc --version
#
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2021 NVIDIA Corporation
Built on Thu_Nov_18_09:45:30_PST_2021
Cuda compilation tools, release 11.5, V11.5.119
Build cuda_11.5.r11.5/compiler.30672275_0
```

### CuPy
[CuPy install](https://docs.cupy.dev/en/latest/install.html)

```bash
#https://docs.cupy.dev/en/latest/install.html#installing-cupy-from-pypi
# CUDA Version v11.2 ~ 11.8 (x86_64 / aarch64)
pip install cupy-cuda11x
```