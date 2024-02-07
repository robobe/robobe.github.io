---
tags:
    - ros
    - dds
    - cyclonedds
    - image
    - large message
---
# Send image message across ROS using cyclonedds

- Ubuntu 22.04
- Humble
- Cyclonedds


## Demo
publish Image 

```xml title="cyclonedds.xml"
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain Id="any">
        <Internal>
            <SocketReceiveBufferSize min="20MB"></SocketReceiveBufferSize>
        </Internal>
    </Domain>
</CycloneDDS>
```

Increased my `/proc/sys/net/core/rmem_max` to 30MB

```bash
echo 30000000 | sudo tee /proc/sys/net/core/rmem_max
```

### use cyclonedds.xml  

```bash
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
```

```bash title="publisher"
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run cpp_demos simple_img_pub
```

```bash title="subscriber"
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  ros2 run cpp_demos simple_img_sub 
```

## Reference
- [ Reduce how eager CycloneDDS is in retransmits #484 ](https://github.com/eclipse-cyclonedds/cyclonedds/issues/484)