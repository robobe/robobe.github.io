---
title: Part3 - Yocto tutorial
description: Run custom image with QEMU
date: "2022-04-06"
banner: ../images/yocto.png
tags:
    - yocto
    - qemu
    - 101
---

```bash
runqemu tmp/deploy/images/qemux86-64/core-image-minimal-qemux86-64-20220406114623.qemuboot.conf
```

```bash
runqemu tmp/deploy/images/qemux86-64/core-image-minimal-qemux86-64-20220406114623.qemuboot.conf nographic
```

!!! Note
    quit `QEMU` nographic mode by press `ctrl-a` then `x`