---
tags:
    - sdf
    - gazebo
    - xacro
    - vscode
---

# SDF

SDFormat (Simulation Description Format), sometimes abbreviated as SDF, is an XML format that describes objects and environments for robot simulators, visualization, and control.
[...](http://sdformat.org/)

---

## Using XACRO to generate sdf
- Using XACRO extension to generate SDF more easily and quickly 
- Using VSCode extension to run xacro tool, every time we save the file.
  - Source file with `sdf.xacro` extension generate `sdf` file

![](images/run_on_save.png)

### config

```json
"emeraldwalk.runonsave": {
        "commands": [
            {
                "match": ".sdf.xacro",
                "cmd": "/opt/ros/humble/bin/xacro ${file} > ${fileDirname}/${fileBasenameNoExt}"
            }
        ]
    }
```

## VSCode settings
### file associations
```json
"files.associations": {
        "*.sdf": "xml",
        "*.model.config": "xml"
    },
```