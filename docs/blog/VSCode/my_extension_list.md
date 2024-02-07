---
tags:
    - vscode
    - extensions
---

# My VSCode extensions list

For python , C++ and dev helper

|                    | category |                                                                              |
| ------------------ | -------- | ---------------------------------------------------------------------------- |
| autoDocstring      | python   | Generates python docstrings automatically                                    |
| Git Graph          | git      | View a Git Graph of your repository, and perform Git actions from the graph. |
| Auto close tag     | xml      | Automatically add HTML/XML close tag                                         |
| Auto Snippet       | dev      | insert a snippet when opening an empty file                                  |
| C/C++              | C++      | C/C++ IntelliSense, debugging                                                |
| C++ TestMate       |          |                                                                              |
| CMake              |          | CMake langage support                                                        |
| CMake Tools        |          |                                                                              |
| Code Runner        | dev      |                                                                              |
| Code Spell Checker | dev      |                                                                              |
| Draw.io            |          |                                                                              |
| Paste Image        | doc      | paste image from clipboard directly                                          |
| Path Intellisense  | dev      |                                                                              |
| Project manager    |          |                                                                              |
| Pylance            |          |                                                                              |
| Pylint             |          |                                                                              |
| Python             |          |                                                                              |
| TabOut             | dev      | Tab out of quotes, brackets, etc                                             |
| Task               | dev      | Load VSCode Tasks into Status Bar                                            |
| TODO Highlight     | dev      | highlight TODOs, FIXMEs, and any keywords, annotations                       |
| XML                | xml      | YAML Language Support                                                        |
| XML Tools          | xml      | XML Formatting, XQuery, and XPath Tools                                      |
| YAML               | yaml     | YAML Language Support                                                        |
| ROS2               | ros      | Syntax highlighting for ros2 interface files                                 |
| docker             |          |                                                                              |
| dev-container      |          |                                                                              |


## todo tree
[todo-tree](https://marketplace.visualstudio.com/items?itemName=Gruntfuggly.todo-tree)

![](images/todo-tree.png)

### Settings
- **todo-tree.filtering.excludeGlobs**: Exclude files or folder from search
- **todo-tree.general.tags**: Add custom tags
- **todo-tree.highlights.customHighlight**: Set tags settings
  - type: line : highlights the entire line containing the tag
  - foreground: 
  - gutterIcon: Set icon in gutter ruler
  - iconColour: Set the colour of the icon in the tree


```json title="settings.json"
   "todo-tree.filtering.excludeGlobs": [
        "**/node_modules/*/**"
    ],
    "todo-tree.general.tags": [
        "BUG",
        "HACK",
        "FIXME",
        "TODO",
        "XXX",
        "[ ]",
        "[x]",
        "CUSTOM"
    ],
    "todo-tree.highlights.customHighlight": {
        "CUSTOM": {
            "iconColour": "#00ff00",
            "type": "line",
            "foreground": "#00ff00"
        },
        "FIXME": {
            "iconColour": "#fff200",
            "gutterIcon": true
        },
        "BUG": {
            "iconColour": "#ff0000",
            "gutterIcon": true,
            "foreground": "#ff0000",
            "type": "line"
        }
    }
```

#### demo
```
"""python
TODO: todo line
- [ ]: check
- [x]: done check
- FIXME: fixme line
BUG: bug to fix
XXX:
CUSTOM: 
"""
def hello_todo():

    print("hello todo")

```