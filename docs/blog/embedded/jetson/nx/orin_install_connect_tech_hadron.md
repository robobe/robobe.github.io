---
tags:
    - jetson
    - nvidia
    - orin
    - sdkmanager
    - connecttech
    - hadron
---
# Hadron
[CTI-L4T Board Support Package Installation for NVIDIA JetPack with Connect Tech Jetson™ Carriers](https://connecttech.com/ftp/Drivers/L4T-Release-Notes/Jetson-Orin-NX-Orin-Nano/ORIN-NX-NANO-36.3.0.pdf)
[kdb373: CTI-L4T Board Support Package Installation for NVIDIA JetPack with Connect Tech Jetson™ Carriers](https://connecttech.com/resource-center/kdb373/)





## Create a minimal rootfs

from Linux_for_tegra folder

```bash
sudo ./tools/samplefs/nv_build_samplefs.sh --abi aarch64 --distro ubuntu --flavor minimal --version jammy
```

```
sudo ./apply_binaries.sh
```
```bash
#sudo ./tools/l4t_create_default_user.sh -u <username> -p <password> -n <machine name>
sudo ./tools/l4t_create_default_user.sh -u user -p user --accept-license
```

## extract connecttech 
- Download CTI-L4T for the jetpack version
- 

## Recovery
- jumper j2
- power up
- wait 10 sec
- remove the jumper
- check with lsusb


## Disable console serial 

```
sudo systemctl disable nv
```

---

## Reference
- [Jetson DeveloperGuide QuickStart](https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/IN/QuickStart.html)
- [Preparing a Jetson to flash and boot an encrypted and non-encrypted image for an SSD](https://developer.ridgerun.com/wiki/index.php/Preparing_a_Jetson_to_flash_and_boot_an_encrypted_and_non-encrypted_image_for_an_SSD)