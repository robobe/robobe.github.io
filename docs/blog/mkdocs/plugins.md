---
title: mkdocs plugins
description: Mkdocs plugins and extensions that i used
date: "2022-04-18"
banner: ../images/mkdocs.png
tags:
    - mkdocs
    - plugins
    - extensions
---
# mkdocs-material
Speak for itself
[Getting started](https://squidfunk.github.io/mkdocs-material/getting-started/)

---

# MkDocs Awesome Pages
The [awesome-pages](https://github.com/lukasgeiter/mkdocs-awesome-pages-plugin) plugin allows you to customize how your pages show up the navigation of your MkDocs without having to configure the full structure in your mkdocs.yml  
For more [info](https://github.com/lukasgeiter/mkdocs-awesome-pages-plugin)

```title="install"
pip install mkdocs-awesome-pages-plugin
```

```title="yml config"
plugins:
    - awesome-pages
```

### Usage
Create `.pages` file in subdirectory to order

```
nav:
    - subdirectory
    - page1.md
    - page2.md
```

!!! Note 
    More examples in project README

---

