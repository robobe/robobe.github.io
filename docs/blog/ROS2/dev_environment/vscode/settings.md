---
title: ROS2 VSCode dev settings
tags:
    - ros2
    - vscode
    - settings
---
## settings
### file associations
```json
"files.associations": {
        "*.xacro": "xml",
        "*.world": "xml",
        "*.sdf": "xml",
        "*.gazebo": "xml"
    }
```

## key bindings
### Add new file and new folder in explorer tree
```json
{
        "key": "ctrl+n",
        "command": "explorer.newFile",
        "when": "explorerViewletFocus"
    },
    {
        "key": "ctrl+shift+n",
        "command": "explorer.newFolder",
        "when": "explorerViewletFocus"
    },
```