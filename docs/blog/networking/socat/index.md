---
title: SOCAT 
description: socat
date: "2022-06-27"
banner: ../images/network.png
tags:
    - socat
    - networking
---


## install
```
sudo apt install socat
```

## usage

```
socat [options] <address> <address>
```

[check this link for more info](https://copyconstruct.medium.com/socat-29453e9fc8a6)


socat is a bidirectional stream protocol by default 
connect two data sources and stream data between them

## Unidirectional Stream
The `-u` flag means that the stream of data runs from the first argument to the second argument  
The `-U` flag reverse the stream

```
socat -u arg1 arg2 is the same as socat -U arg2 arg1.
```


---

# Reference
- [socat Cindy Sridharan](https://copyconstruct.medium.com/socat-29453e9fc8a6)
- [The socat Command in Linux](https://www.baeldung.com/linux/socat-command)
