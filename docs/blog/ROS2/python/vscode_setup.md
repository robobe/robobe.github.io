---
tags:
    - ros2
    - vscode
    - tips
    - python
---

## settings

!!! note colcon build
    ```
    colcon build --symlink-install --merge-install
    ```
     
```json
{
  
  "python.analysis.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/<my_workspace>/install/local/lib/python3.10/dist-packages",
  ]
}

```

| setting  | ext  | description  |
|---|---|---|
| python.analysis.extraPaths  | pylance  | Used to specify extra search paths for import resolution  |

---

## Tips
### Highlight current tab

```json
"workbench.colorCustomizations": {
      "tab.activeBorder": "#ff0000",
      "tab.activeBackground": "#373737"
  }
```


## Keyboard
- Add new file and new folder in explorer view
```json title="keybindings.json"
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

---

## Reference
- [ROS 2 and VSCode](https://picknik.ai/vscode/docker/ros2/2024/01/23/ROS2-and-VSCode.html)