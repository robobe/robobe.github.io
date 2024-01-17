---
tags:
    - vscode
    - tips
---
# VSCode tips and settings

## editor
### Highlight selected file

```
"workbench.colorCustomizations": {
        "tab.activeBorder": "#ff0000"
    },
```

### XML Region
```
<!-- #region -->
...
<!-- #endregion -->
```

---

## tasks
### Global tasks
Place task file at `~/.config/Code/User/tasks.json`


--- 

# XML Region

# Wrap selection
using snippet

#### use case
Add mkdocs text highlight on selection

```json title="snippet"
"mk_text_highlight": {
    "prefix": "mk_h",
    "body": [
    "==${TM_SELECTED_TEXT}=="
    ],
    "description": "highlight text"
}
```

- Add shortcut 

```json title="keyboard shortcut"
{
    "key": "ctrl+h",
    "command": "editor.action.insertSnippet",
    "when": "editorTextFocus",
    "args": {
        "name": "mk_text_highlight",
        "langId": "markdown"
    }
}
```


---

# Extensions

### ignore recommendation
Stop VSCode to check and suggest recommendation ext.

```json
"extensions.ignoreRecommendations": true
```

### extensions.json
Control `recommendations` and `unwantedRecommendations` extensions

- form command pallet select to open `extensions.json`
![](images/configure_recommand_ext.png)