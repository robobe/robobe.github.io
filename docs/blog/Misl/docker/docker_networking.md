---
tags:   
    - docker
    - networking
    - network
---

# Docker Networking
Out of the box docker has three type of networks
  
  

```bash
docker network ls
#
NETWORK ID     NAME      DRIVER    SCOPE
518ee43b204b   bridge    bridge    local
b995620ac824   host      host      local
0ca19644e7f5   none      null      local
```

## Bridge Network
This is the **default** network that container attached by default, the docker engine create a network interface on host usually with an ip address `172.17.0.1/16` and the container get ip address from dhcp config on this interface.
run `docker network inspect bridge` for more information on the configuration and container attach to this network.

## Host Network
Host network use the host's network and not receive an ip address. They are virtually a service spawned on the host’s network and **consume** the host’s ports.

```bash
docker run -it --rm \
--network=host \
busybox
```

## None Network
A none network doesn’t provide any networking capability to the container which means the container is like a black box to the host. The host or any other container won’t be able to communicate with the container.

```bash
docker run -it --rm \
--network=none \
busybox
```

```bash
ifconfig
# only localhost exists 
lo      Link encap:Local Loopback  
        inet addr:127.0.0.1  Mask:255.0.0.0
        ...
```

## MacVLAN Network


```bash
docker network create -d macvlan \
--subnet=192.168.100.0/24 \
--gateway=192.168.100.1 \
-o parent=wlo1 \
pub_net
```

```bash
docker network ls
NETWORK ID     NAME      DRIVER    SCOPE
518ee43b204b   bridge    bridge    local
b995620ac824   host      host      local
0ca19644e7f5   none      null      local
2e404239b3e6   pub_net   macvlan   local
```

```bash
docker run -it --rm \
--network=pub_net \
busybox
```

!!! note 
    `macvlan` can only communicate with other container on the same vlan

     
## Custom bridge

```bash
docker network create -d bridge my_bridge
```

```bash
docker network ls
#
NETWORK ID     NAME        DRIVER    SCOPE
518ee43b204b   bridge      bridge    local
b995620ac824   host        host      local
267224cb8d5c   my_bridge   bridge    local
0ca19644e7f5   none        null      local
```

```bash
docker network inspect my_bridge
#
[
    {
        "Name": "my_bridge",
        "Id": "267224cb8d5cb2d4626549c4082fcb7ff58f14ed67365ddc301de1e1ed8f1b23",
        "Created": "2023-07-06T15:24:31.8552333+03:00",
        "Scope": "local",
        "Driver": "bridge",
        "EnableIPv6": false,
        "IPAM": {
            "Driver": "default",
            "Options": {},
            "Config": [
                {
                    "Subnet": "172.18.0.0/16",
                    "Gateway": "172.18.0.1"
                }
            ]
        },

```

!!! tip DNS
    A bridge network construct its own DNS. The hostname of each container mapped to its IP address


#### Demo
Show using `DNS` on custom bridge network

```bash title="container1"
docker run -it --rm \
--hostname=drone \
--name=done_c \
--network=my_bridge \
busybox
```

```bash title="container2"
docker run -it --rm \
--hostname=ai \
--network=my_bridge \
busybox
```

```bash title="container1"
hostname -i
172.18.0.2

hostname -f
drone

ping ai
64 bytes from 172.18.0.3: seq=0 ttl=64 time=0.120 ms
```

```bash title="container2"
hostname -i
172.18.0.3

hostname  -f
ai

ping drone
PING drone (172.18.0.2): 56 data bytes
64 bytes from 172.18.0.2: seq=0 ttl=64 time=0.100 ms
```

### Add network adapter to container

```bash
docker network connect <network> <container>

```

#### Demo
- Add default bridge to running container with `my_bridge` network

```bash title="container1"
docker run -it --rm \
--hostname=drone \
--name=drone_c \
--network=my_bridge \
busybox
```

```bash title="connect"
docker network connect bridge drone_c
```

```bash
ifconfig 
eth0      Link encap:Ethernet  HWaddr 02:42:AC:12:00:02  
          inet addr:172.18.0.2  Bcast:172.18.255.255  Mask:255.255.0.0
          ...

eth1      Link encap:Ethernet  HWaddr 02:42:AC:11:00:05  
          inet addr:172.17.0.5  Bcast:172.17.255.255  Mask:255.255.0.0
          ...

```

```bash title="disconnect"
docker network disconnect bridge drone_c
```


---

## resources
- [Docker Networking Tutorial // ALL Network Types explained!](https://youtu.be/5grbXvV_DSk)
- [A beginner’s guide to networking in Docker](https://itnext.io/a-beginners-guide-to-networking-in-docker-ca5b822fb935)
- [2 Minutes to Docker MacVLAN Networking – A Beginners Guide](https://collabnix.com/2-minutes-to-docker-macvlan-networking-a-beginners-guide/)


