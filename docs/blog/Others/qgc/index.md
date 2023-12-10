---
tags:
    - qgc
    - qgroundcontrol
---

# Qgroundcontol dev environment
- ubuntu 22.04
- qgc v 4.3.0 (current stable)

!!! note "qgc version"
    Current stable version v 4.3.0 use Qt version 5.12.2
    For support the Qt version we need QtCreate version 4.12
    download qt creator from `https://download.qt.io/archive/qtcreator/4.12/4.12.4/`
     

## clone from github

```bash
# git clone --depth 1 --branch <tag_name> <repo_url>
git clone --depth --branch v4.3.0 https://github.com/mavlink/qgroundcontrol.git
git submodule update --init --recursive
```

## docker
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
- Download qt 5.12.2
- Download qtcreator version 4.12

### Qt
![](images/qt_installer.png)


### Qtcreator
[Download installer](https://download.qt.io/archive/qtcreator/4.12/4.12.4/)



---

## Reference
- [Building using Containers](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/getting_started/container.html)

### Dev
- [Custom Builds    ](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/custom_build/custom_build.html)
- [Hide Settings in QGroundControl Based on User Access Type](https://github.com/godfreynolan/qgccomponenthiding)
- [Customizing Qgroundcontrol ](https://youtu.be/uLRdDl5a-Yk)
    - [github](https://github.com/godfreynolan/qgc_custom)