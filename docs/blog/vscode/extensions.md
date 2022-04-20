---
title: VSCode extensions
description: VSCode plugins and extensions that i used
date: "2022-04-18"
banner: ../images/vscode.jpeg
tags:
    - vscode
    - plugins
    - extensions
---

# PlantUML
PlantUML is an open-source tool allowing users to create diagrams from a plain text language

[PlantUML in a nutshell](https://plantuml.com/)

```bash title="install"
java
graphviz
# 
sudo apt install graphviz
```

```json title="vscode settings"
"plantuml.diagramsRoot": "docs/diagrams/src",
"plantuml.exportOutDir": "docs/diagrams/out",
"plantuml.render": "Local"
```

### usage
- `Alt-D` Preview current diagram
- Create `diagrams/out` and `diagrams/src` folder under `docs` subfolder
- Use `Export current diagram` command to convert diagram to `png`
  
![](images/plant_uml_usage.png)

---

