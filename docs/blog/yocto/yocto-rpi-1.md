---
title: Yocto RPI part 1
description: Raspberry Pi yocto image
date: "2022-04-11"
banner: ../images/yocto-plus-raspberry-pi.png
tags:
    - rpi
    - yocto
---

## clone meta-raspberrypi
```bash
cd meta-external
git submodule add git://git.yoctoproject.org/meta-raspberrypi
# checkout hardknott
```

---

## config

```bash title="bblayers.conf"
POKY_BBLAYERS_CONF_VERSION = "2"

BBPATH = "${TOPDIR}"
BBFILES ?= ""

BBLAYERS ?= " \
  /home/user/yocto/poky/meta \
  /home/user/yocto/poky/meta-poky \
  /home/user/yocto/poky/meta-yocto-bsp \
  /home/user/yocto/meta-external/meta-raspberrypi \
  "
```

```bash title="local.conf"
CONF_VERSION = "1"
MACHINE ?= "raspberrypi4"
BB_NUMBER_THREADS = "6"
PARALLEL_MAKE = "-j 6"
DL_DIR ?= "/home/user/yocto/downloads/"
CONNECTIVITY_CHECK_URIS = "https://www.google.com/"
IMAGE_FSTYPES = "ext4.xz rpi-sdimg" # for SD-CARD
SDIMG_ROOTFS_TYPE = "ext4.xz"
```

## build image
```bash title=""
# rpi-basic-image deprecated
bitbake core-image-base
```

## other packages
```
IMAGE_INSTALL_append = " nano"
```