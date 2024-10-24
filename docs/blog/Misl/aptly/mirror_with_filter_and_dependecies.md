---
tags:   
    - aptly
    - mirror
    - apt
    - offline
---

# Use aptly to mirror selected packages and all the package dependencies
Create repository for selected package and all package dependencies with GPG key

## Demo
Mirror `python3-smbus` package and all the package dependencies for jetson (arm) ubuntu 18.04

### Create
```bash title="create mirror"
aptly mirror create \
-ignore-signatures  \
-architectures=arm64 \
-filter="python3-smbus" -filter-with-deps \
python3-smbus-mirror \
http://ports.ubuntu.com/ubuntu-ports/ bionic
```


### Update
```bash
aptly mirror update -ignore-signatures python3-smbus-mirror
```

### Snapshot

```bash
aptly snapshot create python3-smbus-snap from mirror python3-smbus-mirror
```

### publish

```bash
aptly publish snapshot -skip-signing -distribution=bionic -architectures=arm64 python3-smbus-snap
```

### serve

```bash
aptly serve
#
deb http://lap:8080/ bionic main
```