---
tags:
    - aptly
    - apt
    - repository
    - mirror
---
## LAB - create mirror
Create mirror for qgis, mirror only amd64 architecture  ubuntu jammy 22.04 distribution


#### Mirror
Creates mirror of remote repository

[Aptly documentation for more](https://www.aptly.info/doc/aptly/mirror/create/)

##### Create

```bash title="create mirror"
aptly mirror -architectures="amd64" \
-ignore-signatures \
create qgis-jammy  \
https://ubuntu.qgis.org/ubuntu-ltr/ jammy main 
```

##### Udate
```bash title="update mirror"
aptly mirror \
-ignore-signatures \
update \
qgis-jammy
```


#### Snapshot
```bash title="snapshot"
aptly snapshot create \
qgis-jammy-snapshot \
from mirror qgis-jammy
```

```bash title="create GPG key"
gpg --full-generate-key
#
gpg: key FB7DBD6EA7954594 marked as ultimately trusted
gpg: directory '/home/user/.gnupg/openpgp-revocs.d' created

public and secret key created and signed.

pub   rsa1024 2023-06-10 [SC]
      2061DF2A405A938EFEFC5B93FB7DBD6EA7954594
uid                      robobe <robobe@gmail.com>
sub   rsa1024 2023-06-10 [E]

```

#### Publish
Publishes snapshot as repository to be consumed by apt
Valid GPG key is required for publishing.

[more ](https://www.aptly.info/doc/aptly/publish/snapshot/)

```bash title="publish"
aptly publish \
  snapshot \
  -distribution="jammy" \
  -architectures="amd64" \
  -gpg-key="2061DF2A405A938EFEFC5B93FB7DBD6EA7954594" \
  qgis-jammy-snapshot
```

#### Export key
Export gpg key to be use by apt

```bash title="export key"
gpg --armor --output ~/.aptly/public/gpg --export 2061DF2A405A938EFEFC5B93FB7DBD6EA7954594
```

```bash title="serve"
aptly serve
```

```bash title="install key"
wget -O - -q http://127.0.0.1:8080/gpg | sudo apt-key add -
```

### apt

```bash title="add to source.list"
deb http://127.0.0.1:8080/ jammy main
```

```bash title="apt cache"
apt-cache policy qgis
#
qgis:
  Installed: (none)
  Candidate: 1:3.28.7+36jammy
  Version table:
     1:3.28.7+36jammy 500
        500 http://127.0.0.1:8080 jammy/main amd64 Packages
     3.22.4+dfsg-3build1 500
        500 http://us.archive.ubuntu.com/ubuntu jammy/universe amd64 Packages
```


---

## Reference
- [Setting up a Debian package mirror with Aptly for offline use](https://www.padok.fr/en/blog/debian-mirrors-aptly)