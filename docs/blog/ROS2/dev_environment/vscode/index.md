---
title: ROS2 VSCode settings, Extensions and tips
tags:
    - ros2
    - vscode
---

## files.associations

```json
"files.associations": {
        "*.xacro": "xml",
        "model.config": "xml",
        "*.world": "xml",
        "*.sdf": "xml",
        "*.gazebo": "xml"
    }
```

---

# keyboard settings

```json title=".config/Code/User/keybindings.json"
{
    "key": "ctrl+n",
    "command": "explorer.newFile",
    "when": "explorerViewletFocus"
},
{
    "key": "ctrl+shift+n",
    "command": "explorer.newFolder",
    "when": "explorerViewletFocus"
}
```

---

# Extensions

![](images/xml.png){ align=left }

[xml red hat](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-xml)

```
ext install redhat.vscode-xml
```




