---
title: Part2 - Yocto tutorial
description: Build custom linux image with yocto
date: "2022-04-06"
banner: ../images/yocto.png
tags:
    - yocto
    - poky
    - 101
---

## init environment
```bash
# from poky folder
source oe-init-build-env <location of build folder>
```

create folder and files under chosen `build` location

local.conf
bblayers.conf


## Build
Building custom linux distribution

```bash
# bitbake <image name>
bitbake core-image-minimal
```

!!! Fix
    If bitbake failed to resolve URI  
    set CONNECTIVITY_CHECK_URIS field in `conf/local.conf`

    ```bash title="so 52395512"
    #check connectivity using google
CONNECTIVITY_CHECK_URIS = "https://www.google.com/"

#skip connectivity checks
CONNECTIVITY_CHECK_URIS = ""
    ```