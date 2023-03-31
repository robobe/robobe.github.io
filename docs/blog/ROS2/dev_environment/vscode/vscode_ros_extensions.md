---
title: ROS2 VSCode recommend extensions 
tags:
    - ros2
    - vscode
---

- Language
  - python
  - cpp
  - cmake
- [XML](#xml)
- [yaml](#yaml)
- [dev goodies](#dev-goodies)
  - [auto snippets](#autosnippet)
  - [tabout](#tabout)

---

## yaml

![](images/yaml.png){ align=left }

[YAML red hat](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-yaml)

```
ext install redhat.vscode-yaml
```

---

## XML

![](images/xml.png){ align=left }

[XML red hat](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-xml)

```
ext install redhat.vscode-xml
```

---

## colcon

![](images/colcon.png){ align=left }

[XML red hat](https://marketplace.visualstudio.com/items?itemName=deitry.colcon-helper)

```
ext install deitry.colcon-helper
```

---

## Dev goodies
### Tabout
![](images/tabout.png){ align=left }
[TabOut](https://marketplace.visualstudio.com/items?itemName=albert.TabOut)  
Tab out of quotes, brackets, etc for Visual Studio Code.

### Snippets
![](images/snippets.png){ align=left }
[Snippets](https://marketplace.visualstudio.com/items?itemName=tahabasri.snippets)



### AutoSnippet
![](images/auto-snippet.png){ align=left }
[Auto Snippet](https://marketplace.visualstudio.com/items?itemName=Gruntfuggly.auto-snippet)

```
https://marketplace.visualstudio.com/items?itemName=Gruntfuggly.auto-snippet
```

This extension automatically inserts a predefined snippet when a file is created, or an empty file is opened.

### Demo
Create Template for sdf `model.config` file

```json
"autoSnippet.snippets": [
    {
        "pattern": "**/model.config",
        "snippet": "gazebo_model_config"
    }
]
```

```json title="sdf model config"
"gazebo_model_config": {
    "prefix": "sdf_model_config",
    "body": [
        "<?xml version=\"1.0\"?>",
        "<model>",
        "  <name>${1}</name>",
        "  <version>1.0</version>",
        "  <sdf version=\"${2|1.5,1.6|}\">${3:${1}}.sdf</sdf>",
        "  <author>",
        "    <name></name>",
        "    <email></email>",
        "  </author>",
        "  <description>",
        "  </description>",
        "</model>"
    ],
    "description": "gazebo model config file template"
}
```