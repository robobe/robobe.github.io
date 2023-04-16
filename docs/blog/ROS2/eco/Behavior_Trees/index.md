---
title: Behavior Trees
tags:
    - ros2
    - behavior
---
# Behavior Trees
Is a Task Switching Structure
"What to do next?"

Each action needs to know
"Did i succeed or fail"

![](images/bt.drawio.png)

The Ancestors decide "What to do next"

## Behaviours
- check / condition
- action


### Actions
[check](https://youtu.be/KeShMInMjro?list=PLFQdM4LOGDr_vYJuo8YTRcmv3FrwczdKg&t=104)

- Fallback (?)(or): If failure then tick next else return 
- Sequence (->)(and): if success then tick next else return 

### Condition
Conditions are Action that
- never return Running
- do not change the world


### When to switch task
- Success
- Failure
- Interrupting by more important task


# TO-Read

- [Introduction to Behavior Trees](https://www.youtube.com/playlist?list=PLFQdM4LOGDr_vYJuo8YTRcmv3FrwczdKg)
- [Building a Python Toolbox for Robot Behavior](https://roboticseabass.com/2022/06/19/building-a-python-toolbox-for-robot-behavior/)


# Reference
- [Behavior Trees in Robotics](https://youtu.be/kRp3eA09JkM)
- [Introduction to behavior trees](https://robohub.org/introduction-to-behavior-trees/)