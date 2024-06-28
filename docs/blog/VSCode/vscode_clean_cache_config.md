---
tags:
    - vscode
    - clean
    - cache
---

# todo :
 understand cache and config

```bash
#!/bin/bash

CONFIG_PATH=~/.config/Code 

echo "Configuration Path: $CONFIG_PATH"

for i in "$CONFIG_PATH"/User/workspaceStorage/*; do
    echo "Checking: $i"
    if [ -f "$i/workspace.json" ]; then
        echo "Found workspace.json in $i"
        folder=$(python3 -c "import sys, json; print(json.load(open(sys.argv[1]))['folder'])" "$i/workspace.json" 2>/dev/null | sed 's#^file://##;s/+/ /g;s/%\(..\)/\\x\1/g;')
        echo "Folder extracted: $folder"
        
        if [ -n "$folder" ]; then
            if [ ! -d "$folder" ]; then
                echo "Removing workspace $(basename "$i") for deleted folder $folder of size $(du -sh "$i" | cut -f1)"
                rm -rfv "$i"
            else
                echo "Folder exists: $folder"
            fi
        else
            echo "No folder found in workspace.json"
        fi
    else
        echo "No workspace.json in $i"
    fi
done
```
