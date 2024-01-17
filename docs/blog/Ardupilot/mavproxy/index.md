---
tags:
    - mavproxy
    - mavlink
    - ardupilot
---


## Tips

### Send long command

```bash
module load message
```

![](images/command_long.png)

```bash
message COMMAND_LONG 0 0 198 0 2 0 0 0 0 0 0
```


#### restart lua script's

```bash
module load misc
```

```bash
scripting restart
```