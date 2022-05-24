---
title: YouTube download tool
description: using youtube-dl cli to download from youtube
date: "2022-24-05"
banner: youtube.png
tags:
    - tools
    - youtube
---

## Install
```bash
# uninstall previous version
sudo apt purge youtube-dl
sudo pip3 uninstall youtube-dl
# Download from http://ftp.us.debian.org/debian/pool/main/y/youtube-dl/
wget http://ftp.us.debian.org/debian/pool/main/y/youtube-dl/youtube-dl_2021.12.17-1_all.deb

sudo dpkg -i youtube-dl_2021.12.17-1_all.deb
```

## USage
```bash title="simple"
youtube-dl <video url>
```