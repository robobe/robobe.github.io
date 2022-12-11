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
# DrawIO
![https://marketplace.visualstudio.com/items?itemName=hediet.vscode-drawio](images/drawio.png){ align=left width=150 }
This unofficial extension integrates Draw.io into VS Code.

just create new file named `my_image.drawio.png` or `my_image.drawio.svg`

[VSCode marketplace](https://marketplace.visualstudio.com/items?itemName=hediet.vscode-drawio)
<br>
<br>
<br>

---

# Luna Paint

![](images/luna-paint.png){ align=left width=150}
Luna Paint is a VS Code extension that lets you edit raster images from within the editor, just open an image from the explorer and start editing like any other file.

[VSCode marketplace](https://marketplace.visualstudio.com/items?itemName=Tyriar.luna-paint)

<br>
<br>
<br>

---

# PlantUML
![VSCode marketplace](images/PlantUML.png){ align=left width=150 }

PlantUML is an open-source tool allowing users to create diagrams from a plain text language

[PlantUML in a nutshell](https://plantuml.com/)
<br>
<br>
<br>
<br>
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
