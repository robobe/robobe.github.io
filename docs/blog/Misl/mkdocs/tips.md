---
title: mkdocs tips
description: Mkdocs tips
date: "2022-04-30"
banner: ../images/mkdocs.png
tags:
    - mkdocs
    - tips
---
## image size
```
![](images/mkdocs.png){width=50}
![](images/mkdocs.png){width=150}
```

![](images/mkdocs.png){width=50}

![](images/mkdocs.png){width=150}

---

## image as url link
```
[![](images/mkdocs.png){width=50}](plugins.md)
```

[![](images/mkdocs.png){width=50}](plugins.md)

---

## image align
![](images/mkdocs.png){ align=left width=150}
Text on right side<br>
Text on right side<br>
Text on right side<br>
Text on right side<br>

```
![](images/mkdocs.png){ align=left width=150}
```
---

## Table cell with multiple line
using HTML tag `<br>`

```
| Format   | Tag example                  |
| -------- | ---------------------------  |
| multiline | line1<br>==line2==<br>line3 |
```

| Format   | Tag example                 |
| -------- | --------------------------- |
| multiline | line1<br>==line2==<br>line3 |