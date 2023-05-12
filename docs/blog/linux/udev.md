---
tags:
    - udev
    - linux
    - ubuntu
---
# UDEV custom rules
Udev (userspace /dev) is a Linux sub-system for dynamic device detection and management.
Udev is based on rules
Every received device event is matched against the set of rules read from files located in `/lib/udev/rules.d`

Application and custom rules place under `/etc/udev/rules.f` folder
udev rule files must prefix with number and `.rules` extension.



---

## device info / udevadm info
Query the udev database for device information.


```bash
udevadm info -a -n <device name>
udevadm info -a -n /dev/sda
udevadm info /dev/sda
```
- **-a**: Print all sysfs properties of the specified device that can be used
           in udev rules to match the specified device. It prints all devices
           along the chain, up to the root of sysfs that can be used in udev
           rules.
- **-n**: The name of the device node or a symlink to query (/dev/sda)

---
## custom rule
### rule info query
```bash title="udevadm info"
udevadm info /dev/sda                
P: /devices/pci0000:00/0000:00:14.0/usb3/3-1/3-1.3/3-1.3:1.0/host0/target0:0:0/0:0:0:0/block/sda
N: sda
L: 0
S: disk/by-id/usb-LG_FlashDrive-0:0
S: disk/by-uuid/63AC-7AA5
S: disk/by-path/pci-0000:00:14.0-usb-0:1.3:1.0-scsi-0:0:0:0
S: disk/by-label/my_data
E: DEVPATH=/devices/pci0000:00/0000:00:14.0/usb3/3-1/3-1.3/3-1.3:1.0/host0/target0:0:0/0:0:0:0/block/sda
E: DEVNAME=/dev/sda
E: DEVTYPE=disk
E: DISKSEQ=36
E: MAJOR=8
E: MINOR=0
E: SUBSYSTEM=block
E: USEC_INITIALIZED=3012112447
E: ID_VENDOR=LG
E: ID_VENDOR_ENC=LG\x20\x20\x20\x20\x20\x20
E: ID_VENDOR_ID=0ea0
E: ID_MODEL=FlashDrive
E: ID_MODEL_ENC=FlashDrive\x20\x20\x20\x20\x20\x20
E: ID_MODEL_ID=2168
E: ID_REVISION=2.00
E: ID_SERIAL=LG_FlashDrive-0:0
E: ID_TYPE=disk
E: ID_INSTANCE=0:0
E: ID_BUS=usb
E: ID_USB_INTERFACES=:080650:
E: ID_USB_INTERFACE_NUM=00
E: ID_USB_DRIVER=usb-storage
E: ID_PATH=pci-0000:00:14.0-usb-0:1.3:1.0-scsi-0:0:0:0
E: ID_PATH_TAG=pci-0000_00_14_0-usb-0_1_3_1_0-scsi-0_0_0_0
E: ID_FS_LABEL=my_data
E: ID_FS_LABEL_ENC=my_data
E: ID_FS_UUID=63AC-7AA5
E: ID_FS_UUID_ENC=63AC-7AA5
E: ID_FS_VERSION=FAT16
E: ID_FS_TYPE=vfat
E: ID_FS_USAGE=filesystem

```

|      |                      |
| ---- | -------------------- |
| "P:" | Device path in /sys/ |
| "E:" | Device property      |
| "S:" | Device node symlink  |

[udevadm info output prefixes full list](https://man7.org/linux/man-pages/man8/udevadm.8.html)

### write custom rule

```
SUBSYSTEM=="usb", ACTION=="add", ENV{DEVTYPE}=="usb_device",  RUN+="/usr/local.bin/trigger.sh"
```
- **==** : compare for equality
- **+=** : add value to a key that holds a list of entries.
- **ACTION**
- **SUBSYSTEM**
- **ENV{DEVTYPE}**
- **RUN** : execute a script
- **SYMLINK**: create symlink for the device (under `/dev` folder)


!!! note RUN
    - todo: check again
    Script must location `/usr/local/bin/`, `/bin`, 
     
---

## reload rules
```
udevadm control --reload
```

!!! tip reload rules
    Sometimes it's better to **reboot**
     

---

## Reference
- [An introduction to Udev: The Linux subsystem for managing device events](https://opensource.com/article/18/11/udev)
- [mounting usb automatically](https://unix.stackexchange.com/questions/119973/mounting-usb-automatically-having-usbs-label-as-mountpoint)

