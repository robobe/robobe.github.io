---
title: Systemd minimal service
tags:
    - systemd
    - service
---

## User Scope
How to run systemd service run by unprivileged user

- place the service script in `~/.config/systemd/user`
- run systemctl command with `--user` argument


### demo

```bash
mkdir -p  ~/.config/systemd/user
```

```ini
[Unit]
Description=My new Service

[Service]
Type=simple
ExecStart=/bin/sh -c  'echo "hello service" >> /tmp/my_service.log 2>&1'

[Install]
WantedBy=default.target
```

```bash
systemctl --user enable minimal.service 
systemctl --user daemon-reload
systemctl --user start minimal.service
```

!!! note "system boot"
    OS run the service at boot time only when we set the `WantedBy` to `default.target`
     

---

### systemctl with sudo oneliner

```
echo <pass> | sudo -S <systemctl command>
```
---

# Reference
- [systemd user services and systemctl --user](https://nts.strzibny.name/systemd-user-services/)
- [Understanding systemd at startup on Linux](https://opensource.com/article/20/5/systemd-startup)