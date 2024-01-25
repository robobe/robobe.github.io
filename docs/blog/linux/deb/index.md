---
tags:
    - deb
    - dpkg
    - package
    - debian
---

# DEB Debian package

A Debian package is the simplest and most efficient way of distributing software in Debian-based distributions. It takes care of managing dependencies and provides a good interface for install/upgrade/uninstall operations.

Debian package is an archive that keep the files in specific project structure. 
During the installation the files from the package go into similar folder structure in the target relative to package root folder

```bash
package root
    ├── DEBIAN
    │   ├── control
    │   ├── postinst
    │   └── preinst
    ├── bin
    ├── etc
    ├── usr
    │    └── lib
    └── tmp
```

!!! tip ""
    deb package files follow a specific naming convention
    ```
    <name>_<version>-<revision>_<architecture>.deb
    ```
---

## Simple demo
- Pack bash script
- Use docker for installation test

```
├── Dockerfile
├── install
│   ├── simple_pkg
│   │   ├── DEBIAN
│   │   │   ├── control
│   │   │   └── postinst
│   │   └── usr
│   │       └── local
│   │           └── bin
│   │               └── hello.sh
│   └── simple_pkg.deb
├── README.md
└── scripts
    └── hello.sh
```

- Copy `script/hello.sh` to `install/simple_pkg/usr/local/bin`
- Use `postinst` to add execute permission
- Run `dpkg-deb --build simple_pkg` from install folder
- Build docker
- Run Docker , install and test

```bash title="DEBIAN/control"
Package: simple-pkg
Version: 0.0.1
Section: utils
Priority: optional
Architecture: all
Maintainer: robobe <test@test.com>
Description: simple pkg with bash script
```

```bash title="postinst"
chmod +x /usr/local/bin/hello.sh
```

```dockerfile title="Dockerfile"
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo 'export PS1="🐳 \u@\h: \w\a\ # "' > /home/$USERNAME/.bashrc
```

```bash title="build docker image"
docker build -t helloworld:base --target base .
```

```bash title="docker run"
docker run -it --rm \
--name simple \
--hostname base \
--user user \
-w /home/user \
-v $PWD/install/simple_pkg.deb:/home/user/simple_pkg.deb \
simple_pkg:base \
/bin/bash
```

### install pkg inside docker

```bash title="install pkg"
sudo dpkg -i simple_pkg.deb
```

```bash title="check"
/usr/local/bin/hello.sh
```

---
## Demo package python script
Pack python application for offline install

### Use case
- Install `cherrypy web framework` and all it dependencies

#### 1. Create debian `root` folder

```bash
.
└── simple_pkg
   ├── DEBIAN
   │   ├── control
   │   └── postinst
   ├── opt
   │   └── simple_pkg
   │       └── lib
   │           └── python3.10
   │               └── site-packages
   └── tmp
```

#### 2. Download whl's
- download cherrypy and all dependencies to deb tmp folder

#### 3. Update postinst

```
```

```bash title="DEBIAN/control"
Package: test
Version: 1.0-5
Section: utils
Priority: optional
Architecture: all
Maintainer: robobe <test@test.com>
Description: Python GUI application base on PySimpleGui
Depends: python3-tk
```

```sh title="DEBIAN/postinst"
#!/bin/bash

echo post >> /tmp/test
pip install /tmp/py_gui_demo-0.0.1-py3-none-any.whl
```

```sh title="DEBIAN/preinst"
#!/bin/bash

echo pre >> /tmp/test
```

### build

```bash
dpkg-deb --root-owner-group --build <package root>
```

### check
#### Lintian
Lintian dissects Debian packages and reports bugs and policy violations. It contains automated checks for many aspects of Debian policy as well as some checks for common errors [more](https://manpages.debian.org/stretch/lintian/lintian.1.en.html)

```
sudo apt install lintian
```

```bash
lintian <package.deb>
lintian py_gui_demo_0.0.2_amd64.deb 
```

---

## Install / usage
Using apt to install the deb package and all it's dependencies


```bash title="apt install"
sudo apt --fix-broken install ./test.deb
```

---

## Reference
- [How to Create a Simple Debian Package](https://www.baeldung.com/linux/create-debian-package)