---
title: apt tips
tags:
    - apt
    - linux
    - tips
---

## force override
Force apt-get to overwrite file installed by another package

```
sudo apt-get -o Dpkg::Options::="--force-overwrite" install
```