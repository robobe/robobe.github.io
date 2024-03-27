---
tags:
    - gazebo
    - classic
    - model
    - plugin
    - force
    - pid
    - ros
---

Add PID controller to control position by apply joint force

## Add joint damping

```xml
<joint type="revolute" name="roll">
    <pose relative_to="base_tip">0.5 0 -0.1 0 0 0</pose>
    <child>link3</child>
    <parent>link</parent>

    <physics>
    <ode>
        <implicit_spring_damper>1</implicit_spring_damper>
    </ode>
    </physics>

    <axis>
    <xyz>1 0 0</xyz>
    <dynamics>
        <damping>0.5</damping>
    </dynamics>
    <use_parent_model_frame>1</use_parent_model_frame>
    <limit>
        <lower>-3.14159265</lower>
        <upper>3.14159265</upper>
        <effort>10</effort>
        <velocity>-1</velocity>
    </limit>
    </axis>
</joint>
```

## PID

Read PID controller values from `SDF`
```cpp
sdf::ElementPtr pid = _sdf->GetElement("pid");
double p = 0.1;
if (pid->HasElement("p"))
    p = pid->Get<double>("p");
double i = 0;
if (pid->HasElement("i"))
    i = pid->Get<double>("i");
double d = 0;
if (pid->HasElement("d"))
    d = pid->Get<double>("d");
```

### Plug in pid parameters
```xml
<plugin name='set_joint_position_plugin' filename='libjoint_position_control_plugin.so'>
    <pid>
        <p>0.1</p>
        <i>0</i>
        <d>0.01</d>
    </pid>
</plugin>
```

```cpp title="init pid"
this->pid.Init(p, i, d, 5.0, -5.0);

private:
    gazebo::common::PID pid;
    gazebo::common::Time lastUpdateTime;
```

## PID Cycle
- Calc delta time between cycles
- Get current joint angle 
- Update pid with `dt` and `error`
- Update time


```cpp title="update loop"
void OnUpdate()
{
    auto time = this->model->GetWorld()->SimTime();
    double dt = (this->lastUpdateTime - time).Double();
    auto angle = joint->Position(0);
    double error = angle - set_point;
    double force = this->pid.Update(error, dt);
    this->joint->SetForce(0, force);
    lastUpdateTime = time;
}
```

---

## usage

```
ros2 topic pub -1 \
position/rad \
diagnostic_msgs/msg/KeyValue \
"{key: roll , value: 1}"
```