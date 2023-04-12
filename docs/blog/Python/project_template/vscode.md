---
tags:
    - python
    - project
    - template
    - vscode
---
# Python project template - VSCode
VSCode setting and extensions for python project

## Settings
### Project explorer
```json title="exclude files"
"files.exclude": {
        "venv/": true,
        "**/__pycache__": true,
    }
```

---

## Extensions
### Tasks
![https://marketplace.visualstudio.com/items?itemName=actboy168.tasks](images/tasks_ext.png)
Load VSCode Tasks into Status Bar.

#### Demo
- config isort as task
- `Task` extension add `statusbar` to task config option 
```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "isort",
            "type": "shell",
            "command": "isort ${file}",
            "options": {
                "statusbar": {
                  "color" : "#22C1D6",
                  "tooltip" : "run isort",
                  "label": "isort",
                  "filePattern": "py"
                }
        }
    }
    ]
}
```

!!! note "task useful links"
    - [task Variables Reference](https://code.visualstudio.com/docs/editor/variables-reference)
    - [for task bar icons](https://code.visualstudio.com/api/references/icons-in-labels)
     