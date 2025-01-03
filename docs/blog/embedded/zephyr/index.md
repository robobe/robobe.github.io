---
tags:
    - zephyr
---

# Zephyr
The Zephyr Project is an open-source, real-time operating system (RTOS) designed for resource-constrained devices

## Install
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)

### ubuntu 22.04
#### dependencies
```bash
sudo apt install --no-install-recommends \
    git \
    cmake \
    ninja-build \
    gperf \
    ccache \
    dfu-util \
    device-tree-compiler \
    wget \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-tk \
    python3-wheel \
    python3-venv \
    xz-utils \
    file \
    make \
    gcc \
    gcc-multilib \
    g++-multilib \
    libsdl2-dev \
    libmagic1
```

#### west
Zephyr's meta-tool for project management.

#### west workspace

```bash

```

```bash
pip install west
```



#### Zephyr SDK
The Zephyr SDK contains toolchains for Zephyr’s supported architectures

SDK can install as full version that include all toolchain or as minimal and install request toolchains
[check sdk releases](https://github.com/zephyrproject-rtos/sdk-ng/releases)

---

## VSCode

[mylonics zephyr-ide](https://marketplace.visualstudio.com/items?itemName=mylonics.zephyr-ide)

[Youtube getting stated](https://youtu.be/Asfolnh9kqM)

---

## Reference
- [Zephyr Workbench](https://zephyr-workbench.com/)
- [Zephyr-rtos-tutorial](https://github.com/maksimdrachov/zephyr-rtos-tutorial/tree/main)

---



```
export ZEPHYR_BASE=`pwd`/external/zephyr
```

```
west packages pip --install
west sdk install
```

```
west build -b esp_wrover_kit .
```

```
west build -b native_sim samples/hello_world/
west build -b native_sim --build-dir /tmp/3 samples/hello_world/

#binary found 
# /tmp/3/zephyr/zephyr.exe
```

