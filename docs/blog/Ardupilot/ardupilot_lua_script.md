---
tags:
    - lua
    - ardupilot
    - sitl
    - scripts
---

# Script ardupilot using LUA
ArduPilot has introduced support for Lua scripting. Scripting provides a safe, “sandboxed” environment for new behaviors to be added to the autopilot without modifying the core flight code. Scripts are stored on the SD card and run in parallel with the flight code [more](https://ardupilot.org/copter/docs/common-lua-scripts.html).

To enable scripting set `SCR_ENABLE` parameter to 1 and upload scripts to SD `APM/scripts` folder using mavftp

!!! note "sitl scripts folder"
    lua script locate in `scripts` folder in the root / location of run sitl
     

[lua ardupilot full list](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/docs/docs.lua)


## Install and config

### Lua
[online interpreter](https://www.lua.org/demo.html)


### Config vscode

![https://marketplace.visualstudio.com/items?itemName=sumneko.lua](images/lua_vscode_extension.png)

copy lua docs from [AP_scripting](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/docs/docs.lua) to vscode project root for autocomplete

```
├── docs.lua
├── params
│   └── lua.param
├── README.md
└── scripts
    └── simple_loop.lua
```


### Ardupilot

|   |   |   |
|---|---|---|
| SCR_ENABLE  | enable/disable scripts  | 1: enable  |
| SCR_HEAP_SIZE  |   | usual use default  |

---

### Demo

```lua title="ardupilot lua hello"
local number = math.random()

function update() -- periodic function that will be called
    gcs:send_text(0, "hello lua")

    gcs:send_named_float("lua float", number)
    number = number + math.random()

    -- rescheduler the loop
    return update, 1000
end

-- Run immdeiately before rescheduler
return update()
```

!!! tip "check ardupilot ready state"
    ```lua
    if ahrs:healthy() then
        gcs:send_text(0, "hello lua")
    end
    ```
     
---

## Reference
- [ArduPilot Lua Demos (streamed live, 16 Apr 2022)](https://www.youtube.com/watch?v=UdXGXjigxAo)
- [streamed live code and templates](https://github.com/yuri-rage/lua_live_stream)
- [Ardupilot things to watch](https://www.youtube.com/@MrIampete/videos)
- [Ardupilot lua examples](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/examples)