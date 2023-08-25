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
     
## Demo
Pack python application

```bash
deb
└── build
    └── test_0.0.1_amd64.deb
       ├── DEBIAN
       │   ├── control
       │   ├── postinst
       │   └── preinst
       └── tmp
           └── py_gui_demo-0.0.1-py3-none-any.whl
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