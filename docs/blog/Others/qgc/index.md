---
tags:
    - qgc
    - qgroundcontrol
---

# Qgroundcontol dev environment
- Ubuntu 22.04
- qgc v 4.3.0 (current stable) clone from tag

!!! note "qgc version"
    - QGC Current stable version v 4.3.0
    - Qt version 5.15.2
    - For support the Qt version we need QtCreate version 4.12
    download qt creator from `https://download.qt.io/archive/qtcreator/4.12/4.12.4/`
     

## Clone from github

```bash
# git clone --depth 1 --branch <tag_name> <repo_url>
git clone --depth 1 --branch v4.3.0 https://github.com/mavlink/qgroundcontrol.git
git submodule update --init --recursive
```

## Docker
- build docker image from `deploy/deckerDockerfile-build-linux` docker file
- Create build folder
- Run docker image


```bash
docker build --file ./deploy/docker/Dockerfile-build-linux -t qgc-linux-docker .
# from source root add build folder
mkdir build
docker run --rm -v ${PWD}:/project/source -v ${PWD}/build:/project/build qgc-linux-docker
```

!!! note ""
     The build output locate in `stageing` folder



---

## Build on host
- Download Qt 5.15.2
- Download QtCreator version 4.12

### Qt
Download the [Qt online installer](https://www.qt.io/download-open-source)
Run and select the describe checkbox

![](images/qt_installer.png)


### Qtcreator
[Download installer](https://download.qt.io/archive/qtcreator/4.12/4.12.4/)

![](images/Qtcreator_download.png)

---

## Reference
- [Building using Containers](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/getting_started/container.html)

### Dev
- [Custom Builds    ](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/custom_build/custom_build.html)
- [Hide Settings in QGroundControl Based on User Access Type](https://github.com/godfreynolan/qgccomponenthiding)
- [Customizing Qgroundcontrol ](https://youtu.be/uLRdDl5a-Yk)
    - [github](https://github.com/godfreynolan/qgc_custom)