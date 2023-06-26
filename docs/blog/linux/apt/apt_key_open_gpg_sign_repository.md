---
tags:   
    - apt
    - key
    - gpg
    - pgp
    - sign
    - repository
---

# deb Repository sign
The ubuntu repository sign is a GPG signature that is used to verify the integrity of the packages in the repository.


## verbs
GPG: GNU Privacy Guard
PGP: Pretty Good Privacy
ASC: it is an ASCII-armored format that used to encrypt and sign message with GPG



## Demo: repository key
Demo use ROS humble as repository to use

### key
First download and install key in `/usr/share/keyring` key name following the pattern <repo-name>-archive-keyring.gpg

!!! note "keys"
    apt key can be `gpg` or `asc`
     

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### source.list
- Add deb url to `ros2.list` under `/etc/sources.list.d` folder


```bash
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list 
```

!!! note "signed-by"
    Add key to deb url using `signed-by` attribute
     


!!! note "trusted.gpg"
    `/etc/apt/trusted/gpg` or `/etc/apt/trusted.gpg.d` are the deprecated location to save keys, move key from this files to `/etc/apt/keyrings` location

    
---

## Reference
- [apt-key Is Deprecated. ...](https://www.linuxuprising.com/2021/01/apt-key-is-deprecated-how-to-add.html)
- [apt-key Is Deprecated MUST read ](https://askubuntu.com/questions/1286545/what-commands-exactly-should-replace-the-deprecated-apt-key)