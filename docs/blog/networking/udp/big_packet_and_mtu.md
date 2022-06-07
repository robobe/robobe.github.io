---
title: udp big packet and MTU
description: Detect link MTU by send udp big packets
date: "2022-06-07"
banner: ../images/network.png
tags:
    - udp
    - mtu
---


## Code demo
```python title="udp" linenums="1" hl_lines="1"
--8<-- examples/python/networking/udp/big_Sender.py
```

## Test
- Create ubuntu docker image with tcpdump and netcat

```
FROM ubuntu:20.04
RUN apt-get update && apt-get install --no-install-recommends --yes \
    tcpdump \
    net-tools \ 
    netcat
CMD ["/bin/sh"]
```

### Test 1: Test 123
- Run docker image
- Run tcpdump and sniff for `icmp`
- Send ping from host machine
  
```
tcpdump -n -i eth0 icmp
```

### Test 2: send udp packet less then MTU

```title="docker shell1"
tcpdump -n -i eth0 udp and port 1060 or icmp
```

```title="docker shell2"
nc -ul 1060
```

### Test 3: send udp packet bigger then MTU
- Run tcpdump and nc like in **Test 2**
- send with `sock.setsockopt(socket.IPPROTO_IP, IN.IP_MTU_DISCOVER, IN.IP_PMTUDISC_DO)`
- comment the above line


```bash title="with IP_PMTUDISC_DO"
python examples/python/networking/udp/big_Sender.py
# result
[Errno 90] Message too long
Alas, the datagram did not make it
Actual MTU: 1500

# packet not send at all
```

```bash title="without setsockopt"
python examples/python/networking/udp/big_Sender.py
# Host Result
The big datagram was sent!
# shell 1: tcpdump
19:11:06.400842 IP 172.17.0.1.47327 > 172.17.0.2.1060: UDP, bad length 1500 > 1472

# shell 2: nc
show 1500 char of '#'
```

```
19:14:46.502573 IP 172.17.0.2.1060 > 172.17.0.1.48321: UDP, length 1
19:14:46.502686 IP 172.17.0.1 > 172.17.0.2: ICMP 172.17.0.1 udp port 48321 unreachable, length 37
19:14:51.960248 IP 172.17.0.1.58025 > 172.17.0.2.1060: UDP, bad length 1500 > 1472
```