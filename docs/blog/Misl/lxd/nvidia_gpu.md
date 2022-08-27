---
title: Nvida GPU inside LXD
tags:
    - nvidia
    - lxc
    - lxd
---

## Objective
- Run cuda app inside lxc

## LAB
Using the pervious container add nvidia and cuda capabilities

!!! note
    Nvidia and cuda installed on the host machine

```

lxc config set ubuntu2204 nvidia.runtime=true
lxc config device add ubuntu2204 gpu gpu
# restart container
lxc restart ubuntu2204
# attach shell
lxc exec  ubuntu2204 --user 1000 /bin/bash
# run nvidia-smi
nvidia-smi
```
     
### Test cuda

copy `bandwidthTest` binary from host to container and run

!!! note
    My Host installed with cuda `11.6` version

```bash
lxc file push /usr/local/cuda-11.6/extras/demo_suite/bandwidthTest ubuntu2204/home/ubuntu/

# from container shell run
./bandwidthTest 
[CUDA Bandwidth Test] - Starting...
Running on...

 Device 0: NVIDIA GeForce MX450
 Quick Mode

 Host to Device Bandwidth, 1 Device(s)
 PINNED Memory Transfers
   Transfer Size (Bytes)	Bandwidth(MB/s)
   33554432			3076.7

 Device to Host Bandwidth, 1 Device(s)
 PINNED Memory Transfers
   Transfer Size (Bytes)	Bandwidth(MB/s)
   33554432			3214.5

 Device to Device Bandwidth, 1 Device(s)
 PINNED Memory Transfers
   Transfer Size (Bytes)	Bandwidth(MB/s)
   33554432			70878.8

Result = PASS

```

---

## Reference
- [GPU data processing inside LXD](https://ubuntu.com/tutorials/gpu-data-processing-inside-lxd#1-overview)

