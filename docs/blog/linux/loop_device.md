---
tags:
    - loop device
    - 
---
# loop devices

A loop device can take file and create pseudo block device

using `losetup` command we can create and manage loop devices.


## create loop device
```bash title="create file"
# Create 50M file
dd if=/dev/zero of=blockfile bs=1M count=50
```

```bash title="create loop device"
sudo losetup /dev/loop100 blockfile
```

Create device partition like any other block device

```
sudo parted -s /dev/loop100 mklabel msdos
sudo parted -s /dev/loop100 mkpart primary 1MiB 100%
sudo mkfs.ext4 /dev/loop100p1
```

```bash
# mount
sudo mount /dev/loop100p1 /mnt/loop

# umount
umount /mnt/loop
```

## mount block file

```bash linenums="1" hl_lines="7 9"
sfdisk -d blockfile 
#
label: dos
label-id: 0x7f02b152
device: blockfile
unit: sectors
sector-size: 512

blockfile1 : start=        2048, size=      100352, type=83
```

```bash
echo $((2048 * 512)) $((100352 * 512))
#
1048576 51380224
```

```bash title="mount loop device"
sudo mount -o loop,offset=1048576,sizelimit=51380224 ~/blockfile ~/mnt
```
---

## Reference 
- [How to create loop devices on Linux](https://linuxconfig.org/how-to-create-loop-devices-on-linux)
- [Mounting a partition in a loop device](https://checkmk.com/linux-knowledge/mounting-partition-loop-device)

