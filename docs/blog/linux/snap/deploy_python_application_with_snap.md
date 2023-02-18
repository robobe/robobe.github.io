---
title: Deploy a python application with snapcraft
tags:
    - snap
    - deploy
    - package
    - python
---

Deploy python app as `snap` package

## install
snapcraft program is used to build snaps

```
sudo snap install snapcraft --classic
```

## demo
```
├── setup.py
├── snapcraft.yaml
└── src
    ├── app.py
    ├── data.txt
    └── __init__.py
```

[download](demo.zip)

#### build
Run `snapcraft` from application root folder
```
snapcraft
```

#### install local
!!! tip local install
    Any snap package that is not distributed through the Snap store has to be installed with the –dangerous flag. Install without verifying
     
```bash
sudo snap install --dangerous ./testapp_0.1_amd64.snap 
```

---

## snapcraft.yaml
```yaml
name: testapp
version: '0.1'
summary: python test package
description: |
 test app for python
base: core18
grade: stable
confinement: strict

apps:
  testapp:
    command: bin/hello
    plugs: [home,network-bind]
parts:
  testapp:
    plugin: python
    python-version: python3
    source: .
    stage-packages: [ncbi-blast+]
```


- **Name**: Unique string define the snap name
- **base**:  A base snap is a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications
  - core18: base on ubuntu 18:04




---

# Reference
- [Deploy a Python application with snapcraft](https://dmnfarrell.github.io/software/python-snap)
- [Basic snapcraft.yaml example](https://snapcraft.io/docs/snapcraft-basic-example)