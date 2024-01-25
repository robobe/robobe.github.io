---
tags:
    - teensy
    - udev
    - rule
    - usb
---

# Teensy udev rule

## rule
- Rule contain key-value pair separated by comma
- There are two key types
  - Match key
  - Assignments key
- If all match rule assignments applied 

### Match example
- ACTION
- KERNEL
- SYMLINK
- ATTR
- SUBSYSTEM
- KERNELS, SUBSYSTEMS, DRIVERS, ATTRS, TAGS: These work the same as their singular counterparts, search for matches in the entire upward devpath.
[more]()

### Assignment example
- SYMLINK
- OWNER, GROUP, MODE
- RUN

[]()

```bash
udevadm info -a -p $(udevadm info -a -q path /dev/ttyACM0)
```


```bash
  looking at device '/devices/pci0000:00/0000:00:14.0/usb3/3-4/3-4.1/3-4.1.4/3-4.1.4:1.0/tty/ttyACM0':
    KERNEL=="ttyACM0"
    SUBSYSTEM=="tty"
    DRIVER==""
    ATTR{power/async}=="disabled"
    ATTR{power/control}=="auto"
  ...

  looking at parent device '/devices/pci0000:00/0000:00:14.0/usb3/3-4/3-4.1/3-4.1.4/3-4.1.4:1.0':
    KERNELS=="3-4.1.4:1.0"
    SUBSYSTEMS=="usb"
    DRIVERS=="cdc_acm"
    ATTRS{authorized}=="1"
    ...

  looking at parent device '/devices/pci0000:00/0000:00:14.0/usb3/3-4/3-4.1/3-4.1.4':
    KERNELS=="3-4.1.4"
    SUBSYSTEMS=="usb"
    DRIVERS=="usb"
    ...
    ATTRS{devnum}=="40"
    ATTRS{devpath}=="4.1.4"
    ATTRS{idProduct}=="9802"
    ATTRS{idVendor}=="f055"

```


```
SUBSYSTEM=="usb", ATTRS{idProduct}=="9802", ATTRS{idVendor}=="f055", SYMLINK+="my_teensy"

SUBSYSTEM=="tty", GROUP="dialout", MODE="0660"
```

```bash title="udev run script"
RUN+="/usr/bin/sudo -u juser /home/juser/path/script.sh"
```

## Reload rules and trigger
```bash
sudo udevadm control --reload-rules 
sudo udevadm trigger
```

```bash
ll /dev/my_teensy 
lrwxrwxrwx 1 root root 15 Jan 21 19:00 /dev/teensy -> bus/usb/003/037

```

