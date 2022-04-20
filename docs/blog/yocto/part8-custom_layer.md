---
title: Part8 - Custom layer
description: Raspberry Pi yocto image
date: "2022-04-12"
banner: ../images/yocto-plus-raspberry-pi.png
tags:
    - yocto
    - custom
---
# Custom layer
```
bitbake-layers create-layer <layer name>
```

```bash
tree meta-mylayer
/home/user/yocto/meta-mylayer/
├── conf
│   └── layer.conf
├── COPYING.MIT
├── README
└── recipes-example
    └── example
        └── example_0.1.bb
```

## Add layer to project

```bash
# Add
bitbake-layers add-layer ~/yocto/meta-mylayer/

# Show
bitbake-layers show-layers
NOTE: Starting bitbake server...
layer                 path                                      priority
==========================================================================
meta                  /home/user/yocto/poky/meta                5
meta-poky             /home/user/yocto/poky/meta-poky           5
meta-yocto-bsp        /home/user/yocto/poky/meta-yocto-bsp      5
meta-mylayer          /home/user/yocto/meta-mylayer             6

```

!!! Note
    **Priority**: This is the value used by BitBake to decide which recipe to use and the order in which the .bbappend files should be joined. It means that if two layers include the same recipe (.bb) file, the one with the higher priority is used.

# Add custom recipe
```
meta-mylayer/
├── conf
│   └── layer.conf
├── COPYING.MIT
├── README
├── recipes-example
│   └── example
│       └── example_0.1.bb
└── recipes-hello
    └── hello
        ├── files
        │   └── hello.c
        └── hello_1.0.bb

```

## files
```bash title="recipes-hello.bb"
DESCRIPTION = "Hello-my first recipe"
SECTION = "Mywork"
LICENSE="CLOSED"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"
SRC_URI = "file://hello.c"
S = "${WORKDIR}"
TARGET_CC_ARCH += "${LDFLAGS}"

do_compile() {
         ${CC} hello.c -o hello
}

do_install() {
         install -d ${D}${bindir}
         install -m 0755 hello ${D}${bindir}
}
```

### variables

meta/conf/bitbake.conf

```
export exec_prefix = "/usr"
export bindir = "${exec_prefix}/bin"

TMPDIR ?= "${TOPDIR}/tmp"
WORKDIR = "${TMPDIR}/work/${MULTIMACH_TARGET_SYS}/${PF}"
D = "${WORKDIR}/image"
```

```bash
# TOPDIR
bitbake -e | grep ^TOPDIR=
TOPDIR="/home/user/yocto/poky/build"


```
```c title="hello.c"
#include <stdio.h>

int main()
{
    printf("Hello yocto\n");
    return 0;
}
```

```bash title="conf/local.conf"
CORE_IMAGE_EXTRA_INSTALL += "dropbear \
    python3 \
    hello \
"
```

---

# Build
```
bitbake core-image-minimal
```

---

# Test

```bash
# Run qemu
runqemu qemux86-64 nographic
# login
$ ls -l /usr/bin/hello 
-rwxr-xr-x    1 root     root         14376 Mar  9  2018 /usr/bin/hello

# run program
$ hello
Hello yocto

```

---

# Reference
- [Yocto : Lab 02_Creating Custom layer and writing Recipe for Hello world](https://medium.com/@lokeshsharma596/yocto-lab-02-creating-custom-layer-and-writing-recipe-for-hello-world-f4438311bbfc)