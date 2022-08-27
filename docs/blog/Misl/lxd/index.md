---
title: LXD hello
tags:
    - lxd
    - container
---
LXD system **container** and **virtual machine** manager. 

**Virtual machines** emulate a physical machine, using the hardware of the host system from a full and completely isolated operating system implemented through the use of qemu.  
**System containers**, on the other hand, use the OS kernel of the host system instead of creating their own environment

## Install

```bash
sudo apt update
sudo apt install lxd
# pick version 4.0
```

### Add user to lxd group
```bash
 sudo adduser $USER lxd
 # check with id command
 id
 uid=1000(user) gid=1000(user) groups=1000(user),4(adm),20(dialout),24(cdrom),27(sudo),132(lxd),998(docker)
```

## Init LXD

```
lxd init
```



## Create first container
### launch
launch      Create and start instances from images

```bash
lxc launch images:{distro}/{version}/{arch} {container-name}
```

- launch ubuntu 22.04
```bash
lxc launch images:ubuntu/22.04/amd64 ubuntu2204
```

```bash
lxc list
# result
+------------+---------+--------------------+-----------------------------------------------+-----------+-----------+
|    NAME    |  STATE  |        IPV4        |                     IPV6                      |   TYPE    | SNAPSHOTS |
+------------+---------+--------------------+-----------------------------------------------+-----------+-----------+
| ubuntu2204 | RUNNING | 10.10.10.62 (eth0) | fd42:5179:549e:e81d:216:3eff:fe3f:5b92 (eth0) | CONTAINER | 0         |
+------------+---------+--------------------+-----------------------------------------------+-----------+-----------+

```

### Get shell
```
lxc exec  ubuntu2204 -- /bin/bash
root@ubuntu2204:~# id
uid=0(root) gid=0(root) groups=0(root)

root@ubuntu2204:~# cat /etc/lsb-release 
DISTRIB_ID=Ubuntu
DISTRIB_RELEASE=22.04
DISTRIB_CODENAME=jammy
DISTRIB_DESCRIPTION="Ubuntu 22.04.1 LTS"
```

### login as none root

```bash
lxc exec  ubuntu2204 --user 1000 /bin/bash
# 
ubuntu@ubuntu2204:/$ id
uid=1000(ubuntu) gid=0(root) groups=0(root)

```

---

## Basic lxc

```
lxc list
lxc info <container name>
lxc exec <container name> <command>
lxc stop <container name>
lxc move <container name> < new container name>
lxc copy <container name> < new container name>
lxc delete <container name>
lxc config
lxc file push source <container name>/dest
lxc file pull <container name>/source dest
lxc file edit <container name>/file path
```

alias / custom commands
```
lxc list -c n,s,4,image.description:image

```
---

### login with ssh

- install openssh-server in container

```
sudo apt install openssh-server
```

- Copy host pub key to container
  
```
cp ~/.ssh/id_rsa.pub /tmp/authorized_keys
lxc file push /tmp/authorized_keys ubuntu2204/home/ubuntu/.ssh/authorized_keys -p
```

---

## Snapshot

```
lxc snapshot <container name>
lxc info <container name> # more info about the snapshot
lxc restore <container name> <snap name> 
lxc delete <container name>/<snap name> 
```
---

## sharing
[https://www.cyberciti.biz/faq/how-to-add-or-mount-directory-in-lxd-linux-container/](https://www.cyberciti.biz/faq/how-to-add-or-mount-directory-in-lxd-linux-container/)


```bash
mkdir ~/share
lxc config device add ubuntu2204 share disk source=~/share path=/home/ubuntu/share
# Device share added to ubuntu2204

# show
lxc config device show ubuntu2204
# result
share:
  path: /home/ubuntu/share
  source: /home/user/share
  type: disk

# show on container
ll /home/user/ubuntu
#
...
drwxrwxr-x 2 nobody nogroup 4096 Jul 28 15:42 share/

# fix permission
# 
lxc config set ubuntu2204 raw.idmap "both 1000 1000"

# restart container
lxc restart ubuntu2204
```

---

---

# Reference
- [LXD](https://linuxcontainers.org/lxd/)
- [LXD getting started](https://linuxcontainers.org/lxd/getting-started-cli/)
- [The LXD 2.0: Blog post series [0/12]](https://ubuntu.com/blog/the-lxd-2-0-story-prologue)
- [Install ROS 2 Humble in Ubuntu 20.04 or 18.04 using LXD containers](https://ubuntu.com/blog/install-ros-2-humble-in-ubuntu-20-04-or-18-04-using-lxd-containers)
- [ROS Development with LXD](https://ubuntu.com/blog/ros-development-with-lxd)