---
tags:
    - linux
    - commands
    - update-alternative
---

# update-alternatives

The update-alternatives command use for managing multiple versions of software on Linux systems



## demo
- Add python3 to 

```bash title="install"
#sudo update-alternatives --install <link> <name> <path> <priority>
sudo update-alternatives --install  /usr/bin/python python /usr/bin/python3.10
```


---

## References
- [Update-alternatives Command: A Comprehensive Guide for Linux Users](https://tecadmin.net/linux-update-alternatives-command/)
- [The update-alternatives Command in Linux](https://www.baeldung.com/linux/update-alternatives-command)

