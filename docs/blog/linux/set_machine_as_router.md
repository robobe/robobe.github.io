---
tags:
    - router
    - ubuntu
    - ip_forward
---

# Set machine as Router

- Allow ip_forward
- Add "ip tables" firewall rules


## ip_forward
```
sudo sysctl -w net.ipv4.ip_forward=1

cat /proc/sys/net/ipv4/ip_forward
```

## iptables
```bash
sudo iptables -t nat -A POSTROUTING -o wlo1 -j MASQUERADE
```

```bash
sudo iptables -P FORWARD ACCEPT
```


