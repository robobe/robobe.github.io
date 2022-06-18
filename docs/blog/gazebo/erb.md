---
title: erb
description: using erb templating in gazebo 
tags:
    - erb
    - gazebo
---

ERB is used templating language to generate text file with `Ruby` code


## Generate SDF from ERB template

```bash
erb model.sdf.erb > model.sdf
```

## using cmake 
### project
```
├── build
├── CMakeLists.txt
└── variable_and_math.sdf.erb
└── loops_and_func.sdf.erb
```
### demos

- declare variable
- using `Math` module
  
``` title="variable_and_math.sdf.erb"
--8<-- "examples/gazebo/erb_demo/variable_and_math.sdf.erb"
```

--- 

- loop example
- function and function call

```title="loops_and_func.sdf.erb"
--8<-- "examples/gazebo/erb_demo/loops_and_func.sdf.erb"
```

---

### CMakeLists
```cmake title="CMakeLists.txt"
--8<-- "examples/gazebo/erb_demo/CMakeLists.txt"
```
