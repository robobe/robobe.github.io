---
title: PlotJuggler
tags:
    - ros2
    - plot
    - visualization
    - debugging
---
[PlotJuggler](https://www.plotjuggler.io/) Fast, intuitive and extensible
time series visualization tool.

### install

```
sudo apt install ros-humble-plotjuggler-ros
```

### usage
#### launch

```python
plotjuggler = Node(
        name="kf_plot",
        package="plotjuggler",
        executable="plotjuggler",
        arguments=['-l', os.path.join(pkg, "config", "plot.xml")],
        output="screen"
    )
```

---

# Reference
- [guide](https://facontidavide.github.io/PlotJuggler/visualization_howto/index.html#main-concepts)
- [github](https://github.com/facontidavide/PlotJuggler)

