---
tags:
    - vscode
    - tasks
    - launch
---

# Tasks and Launch variables

[vscode reference](https://code.visualstudio.com/docs/editor/variables-reference#_input-variables)
 
## Variables

[Predefined variables](https://code.visualstudio.com/docs/editor/variables-reference#_predefined-variables)

example demo for open file

```
/home/your-username/your-project/folder/file.ext
```

- ${workspaceFolder} - /home/your-username/your-project
- ${workspaceFolderBasename} - your-project
- ${file} - /home/your-username/your-project/folder/file.ext
- ${fileWorkspaceFolder} - /home/your-username/your-project
- ${relativeFile} - folder/file.ext
- ${relativeFileDirname} - folder
- ${fileBasename} - file.ext


```json title="variable usage"
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "custom task",
            "type": "shell",
            "command": "echo workspace folder: ${workspaceFolder}"
        }
    ]
}
```

---

## User input

- promptString
- pickString


```json title="promptString"
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "custom task",
            "type": "shell",
            "command": "echo Hello, ${input:userName}!"
        }
    ],
    "inputs": [
        {
            "id": "userName",
            "type":"promptString",
            "description": "Enter your name:",
            "default": "World"
        }
    ]
}
```

```json title="pickString"
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "custom task",
            "type": "shell",
            "command": "echo Hello, ${input:userName}!"
        }
    ],
    "inputs": [
        {
            "id": "userName",
            "type":"pickString",
            "description": "Select your option:",
            "options": [
                "option1",
                "option2",
                "option3"
            ],
        }
    ]
}
```

---

## Command variable
Use [Command Variable](https://marketplace.visualstudio.com/items?itemName=rioj7.command-variable#promptstringremember) extension to expand task and launch user input capability

### Demo: selection option with multiple  variable
[rioj7.command-variable](https://marketplace.visualstudio.com/items?itemName=rioj7.command-variable#pickstringremember)

Map option multiple key, value to input variables
each option key value get it's on variable that map to it's **key**


```json
{
    "version": "2.0.0",
    "tasks": [
      {
        "label": "Do some project",
        "type": "process",
        "command": "echo",
        "args": [
          "${input:selectProject.path}",
          "${input:selectProject.name}",
          "${input:selectProject.link}",
          "${input:selectProject.anyOther}"
        ],
        "problemMatcher": []
      }
    ],
    "inputs": [
      {
        "id": "selectProject.path",
        "type": "command",
        "command": "extension.commandvariable.pickStringRemember",
        "args": {
          "key": "path",
          "options": [
            ["project1", {"path":"p1","name":"n1","link":"lnk1","anyOther":"any1"}],
            ["project2", {"path":"p2","name":"n2","link":"lnk2","anyOther":"any2"}]
           ],
          "description": "Pick a project"
        }
      },
      {
        "id": "selectProject.name",
        "type": "command",
        "command": "extension.commandvariable.remember",
        "args": { "key": "name" }
      },
      {
        "id": "selectProject.link",
        "type": "command",
        "command": "extension.commandvariable.remember",
        "args": { "key": "link" }
      },
      {
        "id": "selectProject.anyOther",
        "type": "command",
        "command": "extension.commandvariable.remember",
        "args": { "key": "anyOther" }
      }
    ]
  }
  
```