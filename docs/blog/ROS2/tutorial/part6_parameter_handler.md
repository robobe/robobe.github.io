---
title: Part6 - Simple parameter handler
description: ROS2 parameters handler
date: "2022-04-08"
banner: ../ros2.png
tags:
    - ros2
    - parameters
    - param
    - 101
---


## Run
### List

```bash
ros2 param list
/simple_params:
  my_double_array
  my_int
  my_str
  use_sim_time
```

### get / set
```bash title="get"
# my_str
ros2 param get /simple_params my_str
Parameter not set.

# my_int
ros2 param get /simple_params my_int
Integer value is: 10
```

```bash title="set"
# my_int
ros2 param set /simple_params my_int 20
# result
Set parameter successful

# output log from node callback param function (name, value, type)
[INFO] [1649395615.004278881] [simple_params]: my_int
[INFO] [1649395615.004490765] [simple_params]: 20
[INFO] [1649395615.004679396] [simple_params]: Type.INTEGER

# my_str
ros2 param set /simple_params my_str "world world"
# result
Set parameter successful

# output log from node callback param function (name, value, type)
[INFO] [1649395751.117007430] [simple_params]: my_str
[INFO] [1649395751.117735309] [simple_params]: world world
[INFO] [1649395751.118404909] [simple_params]: Type.STRING

# my_double_array
ros2 param set /simple_params my_double_array "[1.0, 2.0, 3.0]"
# result
Set parameter successful

# output log from node callback param function (name, value, type)
[INFO] [1649396032.647319212] [simple_params]: my_double_array
[INFO] [1649396032.648079309] [simple_params]: array('d', [1.0, 2.0, 3.0])
[INFO] [1649396032.648748882] [simple_params]: Type.DOUBLE_ARRAY

```


## Dump / Load
```bash title="dump"
#ros2 param dump /simple_params --output-dir /tmp
# ros2 param dump /simple_params --print
ros2 param dump /simple_params
# Result
Saving to:  ./simple_params.yaml
```

```bash title="load"
ros2 param load /simple_params simple_params.yaml
# Result 
Set parameter my_double_array successful
Set parameter my_int successful
Set parameter my_str successful
Set parameter use_sim_time successful
```

!!! Note
    `ros2 param load` run the same `param callback` function like `param set`
