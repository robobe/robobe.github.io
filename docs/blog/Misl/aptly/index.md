---
title: Aptly Swiss army knife for Debian repository management 
tags:
    - aptly
    - apt
---

Aptly is a free Debian repository management tool that allows you to mirror remote repositories, manage local package repositories, take snapshots, pull new versions of packages along with dependencies

# LAB - create local repo

```bash
aptly -distribution="focal" -architectures=amd64 repo create my_local_repo
```

!!! warning "gpg"
    using gpg version 1
    replace all the `gpg` command with `gpg1` command
     

     


# Reference
- [tutorials](https://www.aptly.info/tutorial/)