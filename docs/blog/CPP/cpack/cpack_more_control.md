---
tags:
    - cpack
    - cmake
    - deb
    - package
    - dpkg
---

### Version

```
# version
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "2")
set(CPACK_PACKAGE_VERSION_PATCH "0")
```

### Control scripts

```
├── build
├── cmake
│   └── Packing.cmake
├── CMakeLists.txt
├── Debian
│   └── postinst
└── src
    └── hello.cpp
```

```
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/Debian/postinst")
```

!!! note Extract control files from deb
    ```
    dpkg -e <deb> <extract location>
    ```
     

### Dependencies

```
set(CPACK_DEBIAN_PACKAGE_DEPENDS "")
```

!!! note check dependencies using dpkg 
    ```
    # Check **Depend** section
    dpkg -I <deb>
    ```