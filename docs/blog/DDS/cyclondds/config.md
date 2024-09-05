RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$PWD/dds.xml" ros2 topic list


RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$PWD/dds.xml" ros2 topic pub  /greetings std_msgs/msg/String "{data: 'Hello from terminal'}"

<?xml version="1.0" encoding="utf-8"?>
<CycloneDDS
  xmlns="https://cdds.io/config"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd"
>
  <Domain Id="any">
    <General>
      <Interfaces>
            <NetworkInterface name="eth0"/>
        </Interfaces>
</General>

  </Domain>
</CycloneDDS>


https://cyclonedds.io/docs/cyclonedds/latest/config/index.html

https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html


---

# test 1

```bash title="jetson"
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 topic pub  /greetings std_msgs/msg/String "{data: 'Hello from terminal'}"
```

```
ros2 topic echo /greetings
```

---

# test 2

```bash title="pc"
ros2 topic pub  /greetings std_msgs/msg/String "{data: 'Hello from terminal'}"
```

```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  ros2 topic echo /greetings
```