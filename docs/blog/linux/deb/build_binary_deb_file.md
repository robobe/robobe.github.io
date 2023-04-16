---
tags:
    - deb
    - ubuntu
    - package
---
# Build binary deb package
A deb is a standard Unix ar archive that contains your application and other utility files.


Internally, a deb package contains a collection of folders that **mimics** a typical Linux file system, such as `/usr`, `/usr/bin`, `/opt` and so on. A file put in one of those directories will be copied to the same location in the actual file system during installation

deb package files follow a specific naming convention

```
<name>_<version>-<revision>_<architecture>.deb
```


## Tutorial

Build deb package for cpp application
 

```cpp title="app"
#include <iostream>

int main(int argc, char** argv){
    std::cout << "hello debian package" << std::endl;
    return 0;
}
```

```bash title="build"
g++ hello.cpp -o hello
```

### Create temporary working directory

```bash 
mkdir hello_1.0_1_amd64
mkdir -p hello_1.0_1_amd64/usr/local/bin
cp hello hello_1.0_1_amd64/usr/local/bin/
```

### control file
```
mkdir hello_1.0_1_amd64/DEBIAN
touch hello_1.0_1_amd64/DEBIAN/control
```

```title="DEBIAN/control"
Package: hello
Version: 1.0
Architecture: amd64
Maintainer: Internal Pointers 
Description: A program that greets you.

```

---

### Build

```bash
# dpkg-deb --build --root-owner-group <package-dir>
dpkg-deb --build --root-owner-group hello hello_1.0_1_amd64
```

---

### Test

```
sudo dpkg -i hello_1.0_1_amd64.deb 
```

```
dpkg -l | grep hello
```

```
sudo dpkg -r hello
```

---

### scripts

Run scripts before or after package installation and removal
script locate in `DEBIAN` sub folder

!!! note "permission"
    Don't forget execute permission

- preinst: before install
- postinst: post install
- prerm: before remove
- postrm: after remove
     
---

### dependencies

Add dependencies to `Depends` section

```bash
# from deb tmp folder
# create empty debian/control file
# use dpkg-shlibdeps to get binary dependencie
# Add them to DEBIAN/control Depends section

dpkg-shlibdeps -O path/to/binary/file
```

---

# Reference
- [Building binary deb packages: a practical guide](https://www.internalpointers.com/post/build-binary-deb-package-practical-guide)
- [The Debian Archive](https://www.debian.org/doc/debian-policy/ch-archive.html#s-subsections)