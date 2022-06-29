---
title: SOCAT serial
description: socat serial
date: "2022-06-27"
banner: ../images/network.png
tags:
    - socat
    - serial
---

# demo

```bash
# server (udp-serial)
socat -d -d -d udp-listen:1234 open:/dev/ttyUSB0,raw,nonblock,waitlock=/tmp/s0.lock,echo=0,b115200,crnl

# client
socat - UDP:127.0.0.1:1234


socat -d -d -d UDP-RECVFROM:1234,fork open:/dev/ttyUSB0,raw,nonblock,waitlock=/tmp/s0.lock,echo=0,b115200,crnl
```

---

!!! note
    TTY ports are direct connections to the computer such as a keyboard/mouse or a serial connection to the device. PTS connections are SSH connections or telnet connections  
    [Difference between /dev/tty and /dev/pts (tty vs pts) in Linux](https://www.golinuxcloud.com/difference-between-pty-vs-tty-vs-pts-linux/)
     
### serial echo server
```bash title="server"
socat -d -d pty,raw,echo=0  exec:'/bin/cat',pty,raw,echo=0
# output
N PTY is /dev/pts/6
socat[117907] N forking off child, using pty for reading and writing
socat[117907] N forked off child process 117908
socat[117907] N forked off child process 117908
socat[117907] N starting data transfer loop with FDs [5,5] and [7,7]
socat[117908] N execvp'ing "/bin/cat"
```

```bash title="client"
socat - /dev/pts/6
```