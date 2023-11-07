---
tags:
    - lua
    - cheat sheet
---


### condition

```lua
-- canVote = age > 18 ? true : false
canvote = age > 18 and true or false
```

---
### Strings



### loops

```
i = 1
while (i <= 10) do
    io.write(i)
    i = i + 1
    if i == 8 then break end
end
```
---

### tables
!!! tip "table are one base"
    table are one base and not zero base like many programming language
     
mytable = { 'name_a', 'name_b' }

   
for k, v in pairs(mytable) do
    print(k, v)
end

print(mytable[1])
print(mytable[2])


---

### files

```lua
file = io.open("test.txt", "w+")
file:write("first line\n")
file:write("second line\n")

file:seek("set", 0)

print(file:read("*a"))

file:close()

```

---

### modules

```
├── lib
│   └── convert.lua
├── model_demo.lua
```

```lua title="lib/convert.lua"
local convert = {}

function convert.cm2in(cm)
    return 2.54 * cm
end

return convert
```

```lua title="model_demo"
convert_module = require("examples.lib.convert")  -- same directory

print(string.format("%f", convert_module.cm2in(2)))
```

---

### oop

Lua isn't oop language
We create object like with tables and metatables

```lua
Animal = {name="", color="black"}

function Animal:new(name)
    setmetatable({}, Animal)
    self.name = name

    return self
end

function Animal:toString()
    animal_str = string.format("%s %s", self.name, self.color)
    return animal_str
end
```

```lua title="usage"
spot = Animal:new(10)
print(spot.color)
print(spot:toString())
```