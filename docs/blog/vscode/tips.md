---
title: VSCode tips
description: VSCode tips
date: "2022-04-30"
banner: ../images/vscode.jpeg
tags:
    - vscode
    - tips
---

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

# XML Region
```
<!-- #region -->
...
<!-- #endregion -->
```