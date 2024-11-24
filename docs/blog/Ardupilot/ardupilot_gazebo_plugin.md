---
tags:
    - gazebo
    - harmonic
    - ardupilot
    - plugin
    - sitl
---

# Ardupilot Gazebo Plugin

```xml
<control channel="0">
    <jointName>left_chasis_engine_joint</jointName>
    <useForce>0</useForce>
    <multiplier>1</multiplier>
    <offset>0</offset>
    <servo_min>1100</servo_min>
    <servo_max>1900</servo_max>
    <type>POSITION</type>
    <p_gain>0.20</p_gain>
    <i_gain>0</i_gain>
    <d_gain>0</d_gain>
    <i_max>0</i_max>
    <i_min>0</i_min>
    <cmd_max>2.5</cmd_max>
    <cmd_min>-2.5</cmd_min>
    <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
</control>
```

### type
- command
- effort
- velocity

```cpp
const double pwm = _pwm[this->dataPtr->controls[i].channel];
const double pwm_min = this->dataPtr->controls[i].servo_min;
const double pwm_max = this->dataPtr->controls[i].servo_max;
const double multiplier = this->dataPtr->controls[i].multiplier;
const double offset = this->dataPtr->controls[i].offset;

double raw_cmd = (pwm - pwm_min)/(pwm_max - pwm_min);
raw_cmd = gz::math::clamp(raw_cmd, 0.0, 1.0);
this->dataPtr->controls[i].cmd = multiplier * (raw_cmd + offset);
```

# Demo

Servo position 180 degree

| servo / pwm | Degree |
| ----------- | ------ |
| 1000        | 0 (0)      |
| 1500        | pi/2 (90)      |
| 2000        | pi (180)      |


$$
raw\_cmd = \frac{1500 - 1000}{2000-1000}
$$

```
multiplier * (raw_cmd + offset);
```
$$
cmd = pi * (raw\_cmd + 0)
$$

|  raw_cmd | cmd  |
|---|---|
|  0 |  0 |
|  $\frac{1}{2}$ | 1.575  |
|  1 | 3.14  |

---

## Demo2

| servo / pwm | Degree |
| ----------- | ------ |
| 1000        | -pi/2 (-90)      |
| 1500        | 0 (0)      |
| 2000        | pi/2 (90)      |

$$
raw\_cmd = \frac{pwm - 1000}{2000-1000}
$$

```
# raw_cmd : 0 - 1
multiplier * (raw_cmd + offset);
pi * (raw_cmd - 0.5)
```

---

### ApplyMotorForce
- command: Publish cmd to other plugin
- effort
- velocity
- position

#### command

#### useForce flag
Joint get force command from pid

##### velocity
TODO

##### position
Get pos target from calc pwm, get current pose, 
Set force command from PID output output
##### effort
Set calc command without pid, directly to joint force
