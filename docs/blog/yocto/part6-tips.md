---
title: Part6 - Tips
description: tips
date: "2022-04-11"
banner: ../images/yocto.png
tags:
    - yocto
    - 101
---

# Sharing downloads

```
DL_DIR ?= "/home/user/yocto/downloads/"
```

# Fix Resolve
```
CONNECTIVITY_CHECK_URIS = "https://www.google.com/"
```

# parallel
```bash
# Determines the number of tasks that BitBake will perform in parallel (for bitbake command)
BB_NUMBER_THREADS = "6"
# Determine the number of processes that make can run in parallel (for make command)
PARALLEL_MAKE = "-j 6"
```