---
tags:
    - nvidia
    - jetson
    - cuda
---

## install on jetson

```bash
Copy "cuda-repo-l4t-11-4-local_11.4.14-1_arm64.deb" file to target Orin
$ sudo dpkg -i cuda-repo-l4t-11-4-local_11.4.14-1_arm64.deb
$ sudo apt-key add /var/cuda-repo-11-4-local-xxxxx.pub
$ sudo apt-get update
$ sudo apt-get -y install cuda-toolkit-11-4
```


## Hello world

```cpp title="hello.cu"
#include <stdio.h>

__global__ void helloCUDA()
{
    printf("Hello, CUDA!\n");
}

int main()
{
    helloCUDA<<<1, 1>>>();
    cudaDeviceSynchronize();
    return 0;
}
```

```bash
nvcc hello.cu -o hello
```

!!! tip "cu extansion"
    . cu extension if you want nvcc (the cuda compiler) to recognise it as a cuda file and compile it accordingly

In CUDA, the __global__ prefix is used to declare a kernel function that runs on the GPU.


`<<< >>>`: This is the kernel launch syntax, which is used to specify the execution configuration of the kernel.

`cudaDeviceSynchronize()` is used to ensure that all GPU operations are completed before the CPU continues executing.

---

## Reference
- [cuda basic](https://www.nvidia.com/docs/io/116711/sc11-cuda-c-basics.pdf)
