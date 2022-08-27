---
title: Create minimal rootfs for embedded device like jetson nano and rpi
tags:
    - rootfs
    - embedded
---

## step 1 - minimal root

```bash
# debootstrap --arch $ARCH $RELEASE  $DIR [$MIRROR]
# debootstrap --arch=arm64 focal rootfs
debootstrap --arch=arm64 --foreign --verbose focal rootfs
```

!!! note "ubuntu versions"
    | Version | code name       |
    | ------- | --------------- |
    | 22.04   | Jammy Jellyfish |
    | 20.04   | Focal Fossa     |
    | 1804    | Bionic Beaver   |

     
**--foreign:**
    Do the initial unpack phase of bootstrapping only, for example if the target architecture does not match the host architecture.  The first stage downloads the needed .deb files and unpacks them into the directory you specify

**--verbose**
    Produce more info about downloading.


### second-stage
The second stage runs all of the package configuration scripts, which must be done using the target architecture (or by using qemu-user-static to emulate the target architecture).

!!! tip "qemu"
    ```bash title="install"
    sudo apt install qemu-static-user binfmt-support
    ```

    ```bash title="copy into rootfs"
    install -Dm755 $(which qemu-aarch64-static) rootfs/usr/bin/qemu-aarch64-static
    ```
     
### mount

!!! note "bind"
    A bind mount takes an existing directory tree and replicates it under a different point. 

    ```
    mount --bind /some/where /else/where
    ```
     
---

# Reference
- [nvidia minimal RootFS](https://forums.developer.nvidia.com/t/minimal-rootfs/173232)
- [Jetson Linux](https://developer.nvidia.com/embedded/jetson-linux)
- [Rolling your own minimal embedded Linux for the Raspberry Pi](https://kevinboone.me/pi_boot_shell.html)
