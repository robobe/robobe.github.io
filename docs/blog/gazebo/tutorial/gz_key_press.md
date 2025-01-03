---
tags:
    - gazebo
    - key
    - sdf
---

# Control simulation using keys

```xml
<!-- listen to keyup and pub twist message-->
<plugin filename="gz-sim-triggered-publisher-system"
    name="gz::sim::systems::TriggeredPublisher">
    <input type="gz.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">16777235</match>
    </input>
    <output type="gz.msgs.Twist" topic="/cmd_vel">
        linear: {x: 0.5}, angular: {z: 0.0}
    </output>
</plugin>
```

```xml
<!--add keyPublisher plugin to gui section-->

<gui>
    <plugin filename="KeyPublisher" name="key publisher"></plugin>
</gui>

```