---
tags:
    - aptly
    - repo
    - local
---

# aptly repo
Local repository is a collection of packages (most usually custom packages created internally). 
Multiple versions of the same package could be added to the repository. In order to capture current repository state we use snapshot

## repo
### Create
Create local package repository. Repository would be empty when created, packages could be added to the repository from local files, copied or moved from another local repository or imported from the mirror.
[more](https://www.aptly.info/doc/aptly/repo/create/)


```bash
aptly -distribution="jammy" -architectures="amd64" \
repo \
create \
my_repo
```

### List
Commands list displays list of all local package repositories.
[more](https://www.aptly.info/doc/aptly/repo/list/)

```bash
aptly repo list
```

### Add
Command adds packages to local repository from .deb
[more](https://www.aptly.info/doc/aptly/repo/add/)


From `deb` folder location

```bash
aptly repo add my_repo *deb
```     

---

## Snapshot
Snapshot is a fixed state of remote repository mirror or local repository.
Internally snapshot is list of references to packages. 

[more](https://www.aptly.info/doc/aptly/snapshot/)

Creates snapshot from current state of local package repository. 

```bash
aptly snapshot create <name> from repo <repo-name>
```
- name: snapshot name
- repo-name: local repository name


```bash
aptly snapshot create \
snap_my_repo \
from repo my_repo
```

---

## Publish
Publishes snapshot as repository to be consumed by apt. 
[more](https://www.aptly.info/doc/aptly/publish/snapshot/)

```bash
aptly publish snapshot <name> [<prefix>]
```

- **prefix**: Add prefix to repo url
- **architectures**: publish only selected arc from snapshot
- **distribution**: distribution name to publish
- **skip-signing**: don’t sign Release files with GPG

```bash title="publish with prefix"
aptly -architectures="amd64" -skip-signing=true \
publish snapshot -architectures="amd64" \
snap_my_repo local
```

---

## Serve
aptly can serve published repositories using its own embedded webserver.
[more](https://www.aptly.info/doc/aptly/serve/)

```bash
aptly server
```

### source.list
- Add this line to use by apt


```bash title="line in source.list"
deb http://lap2:8080/local/ jammy main
```

---

## Reference
- [Introducing ‘Aptly’, A Debian Repository Management Tool](https://www.unixmen.com/introducing-aptly-a-debian-repository-management-tool/)