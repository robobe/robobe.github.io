---
title: Adding Latency and Limiting Bandwidth
description: tc - Adding Latency and Limiting Bandwidth
date: "2022-04-25"
banner: images/linux.jpg
tags:
    - tc
    - networking
    - linux
---

HTB help you to control the use of the outbound bandwidth on a given link

```
ping 8.8.8.8
PING 8.8.8.8 (8.8.8.8) 56(84) bytes of data.
64 bytes from 8.8.8.8: icmp_seq=1 ttl=115 time=67.2 ms
64 bytes from 8.8.8.8: icmp_seq=2 ttl=115 time=67.6 ms
```

```bash
INTERFACE=wlo1
IP=8.8.8.8/32

sudo tc qdisc add dev $INTERFACE root handle 1:0 htb default 10
# class
sudo tc class add dev $INTERFACE parent 1:0 classid 1:10 htb rate 1024mbit
# latency
sudo tc qdisc add dev $INTERFACE parent 1:10 handle 10:0 netem delay 200ms

# filter
sudo tc filter add dev $INTERFACE protocol ip parent 1:0 prio 1 u32 match ip dst $IP flowid 1:10
```

```bash title="result"
# ping 8.8.8.8
ping 8.8.8.8
PING 8.8.8.8 (8.8.8.8) 56(84) bytes of data.
64 bytes from 8.8.8.8: icmp_seq=1 ttl=115 time=267 ms
64 bytes from 8.8.8.8: icmp_seq=2 ttl=115 time=267 ms
64 bytes from 8.8.8.8: icmp_seq=3 ttl=115 time=268 ms

# ping google.com
# resolve domain with dig `dig google.com`
ping 142.250.186.46
PING 142.250.186.46 (142.250.186.46) 56(84) bytes of data.
64 bytes from 142.250.186.46: icmp_seq=1 ttl=114 time=84.4 ms
64 bytes from 142.250.186.46: icmp_seq=2 ttl=114 time=80.7 ms
64 bytes from 142.250.186.46: icmp_seq=3 ttl=114 time=82.6 ms

```

```bash title="remove"
sudo tc qdisc del dev $INTERFACE root
```
# Reference
- [Adding Latency and Limiting Bandwidth](http://blog.tinola.com/?e=22)