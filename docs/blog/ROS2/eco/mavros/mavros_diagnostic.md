---
title: mavros diagnostic
tags:
    - mavros
    - diagnostic
    - rat_runtime_monitor
---

!!! note ""
     see [mavros_ardupilot_sitl_hello](mavros_ardupilot_sitl_hello.md) to run sitl and mavros node

## Run time monitor

```
apt install ros-humble-rqt-runtime-monitor
```

### usgae

![](images/run_time_monitor.png)

## check the code

```cpp title="" linenums="1" hl_lines="55"
  void handle_heartbeat(
    const mavlink::mavlink_message_t * msg,
    mavlink::minimal::msg::HEARTBEAT & hb, plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    using mavlink::minimal::MAV_MODE_FLAG;

    // XXX(vooon): i assume that UAS not interested in HBs from non-target system.

    // Store generic info of all heartbeats seen
    auto it = find_or_create_vehicle_info(msg->sysid, msg->compid);

    auto vehicle_mode = uas->str_mode_v10(hb.base_mode, hb.custom_mode);
    auto stamp = node->now();

 

    // update context && setup connection timeout
    uas->update_heartbeat(hb.type, hb.autopilot, hb.base_mode);
    uas->update_connection_status(true);
    timeout_timer->reset();

    // build state message after updating uas
    auto state_msg = mavros_msgs::msg::State();
    state_msg.header.stamp = stamp;
    state_msg.connected = true;
    state_msg.armed = !!(hb.base_mode & enum_value(MAV_MODE_FLAG::SAFETY_ARMED));
    state_msg.guided = !!(hb.base_mode & enum_value(MAV_MODE_FLAG::GUIDED_ENABLED));
    state_msg.manual_input = !!(hb.base_mode & enum_value(MAV_MODE_FLAG::MANUAL_INPUT_ENABLED));
    state_msg.mode = vehicle_mode;
    state_msg.system_status = hb.system_status;

    state_pub->publish(state_msg);
    hb_diag.tick(hb.type, hb.autopilot, state_msg.mode, hb.system_status);
  }
```

### state message

![](images/mavlink_heartbeat.png)

```bash
# find state topic
ros2 topic list | grep state
#
/mavros/extended_state
/mavros/state

# get more info on the topic
ros2 topic info /mavros/state
#
Type: mavros_msgs/msg/State
Publisher count: 1
Subscription count: 0

ros2 interface show mavros_msgs/msg/State
#
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
bool connected
bool armed
bool guided
bool manual_input
string mode
uint8 system_status

```