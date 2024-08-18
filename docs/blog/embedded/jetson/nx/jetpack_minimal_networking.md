---
tags:
    - netplan
    - systemd-networkd
    - network-manager
---

# Config Jetson using netplan and systemd-networkd

## systemd-networkd

```bash
#check if networkd manager the network

networkctl

IDX LINK    TYPE     OPERATIONAL SETUP    
  1 lo      loopback carrier     unmanaged
  2 l4tbr0  bridge   off         unmanaged
  3 usb0    gadget   no-carrier  unmanaged
  5 can0    can      off         unmanaged
  6 eth0    ether    routable    unmanaged
```

```bash title="systemd-networkd"
# start and enabled
sudo systemctl unmask systemd-networkd
sudo systemctl enable systemd-networkd
sudo systemctl start systemd-networkd
```

```bash title="NetworkManager"
# stop and disabled
sudo systemctl stop NetworkManager
sudo systemctl disable NetworkManager
sudo systemctl mask NetworkManager
```

```
## netplan

```
sudo apt install netplan.io
```

```yaml title="netplan config example"
network:
    version: 2
    renderer: networkd
    ethernets:
        enp3s0:
            dhcp4: true
            dhcp6: true
    wifis:
        wlp2s0b1:
            dhcp4: yes
            dhcp6: yes
            access-points:
                "network_ssid_name":
                    password: "**********"
```

```yaml title="static"
network: 
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: false
      dhcp6: false
      addresses:
      - 10.0.0.4/24
      routes:
      - to: default
        via: 10.0.0.1
      nameservers:
       addresses: [8.8.8.8,8.8.4.4]
```

```bash title="apply configuration"
sudo netplan apply
```
```bash
networkctl 
IDX LINK    TYPE     OPERATIONAL SETUP     
  1 lo      loopback carrier     unmanaged
  2 l4tbr0  bridge   off         unmanaged
  3 usb0    gadget   no-carrier  unmanaged
  4 usb1    gadget   no-carrier  unmanaged
  5 can0    can      off         unmanaged
  6 eth0    ether    routable    configured
  7 docker0 bridge   no-carrier  unmanaged
```
