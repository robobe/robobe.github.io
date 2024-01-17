---
tags:
    - ardupilot
    - lua
    - script
---
ArduPilot has introduced support for Lua scripting. Scripting provides a safe, “sandboxed” environment for new behaviors to be added to the autopilot without modifying the core flight code. Scripts are stored on the SD card and run in parallel with the flight code. for more [check](https://ardupilot.org/dev/docs/common-lua-scripts.html) 


!!! tip 
    From MAVProxy we can upload lua scripts using `ftp` module
    We can restart / stop script using module `misc` that include scripting stop and restart function

    ```
    scripting restart
    ```
     

## Demo
[ardupilot lua scripts example](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/examples)

```lua
-- This script is an example of saying hello.  A lot.
-- Pick a random number to send back
local number = math.random()

function update() -- this is the loop which periodically runs
  gcs:send_text(0, "hello, world") -- send the traditional message

  gcs:send_named_float('Lua Float',number) -- send a value
  number = number +  math.random() -- change the value

  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
```

### upload and restart script using mavproxy
```bash
STABILIZE>ftp put <source path>/hello.lua scripts/hello.lua
STABILIZE> Putting hello.lua as scripts/hello.lua
Sent file of length  481

STABILIZE> scripting restart
STABILIZE> Got COMMAND_ACK: SCRIPTING: ACCEPTED
AP: Scripting: stopped
Lua: State memory usage: 4824 + 26372
AP: Scripting: restarted
AP: hello, world
AP: hello, world
AP: hello, world
```

## Demo2

!!! warning usin require not support
     https://discuss.ardupilot.org/t/how-to-import-another-lua-script-to-main-lua-script/102752/3

     The demo work for me 

- using helper module
- create directory for lua_modules
- upload/put helper.lua at `lua_modules` directory

!!! tip other scripts
    Must be not in `scripts` folder
     

```bash
ftp mkdir lua_modules
```

```lua title="helper.lua" linenums="1" hl_lines="1,7"
helper = {}

function helper.get_message()
    return "hello 2"
end

return helper
```

```lua title="hello.lua:  linenums="1" hl_lines="2,5"

local obj = require("lua_modules/helper")

function update()
  gcs:send_text(0, obj:get_message())


  return update, 1000
end

return update()
```

---

- [ardupilot lua](https://ardupilot.org/dev/docs/common-lua-scripts.html)
- [yuri-rage ArduRover_Mower](https://github.com/yuri-rage/ArduRover_Mower/tree/master)
- [ArduPilot Lua Demos](https://youtu.be/UdXGXjigxAo)
    - [github](https://github.com/yuri-rage/lua_live_stream)