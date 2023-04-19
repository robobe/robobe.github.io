---
tags:
    - vscode
    - marpit
    - presentation
    - marp
    - markdown
    - mermaid
---
# Marpit: Markdown presentation


## Usage
```markdown title="slides.md"
---
marp: true
size: 4:3
---

# Headings

## Heading 2
### Heading 3

---

# Basic Formatting

This should be **bold** and *italic*.

**Unorder list**
- item 1
- item 2
- item 3

---

**Order list**
1. item 1
1. item 2
1. item 3
```

### Preview
!!! tip "View Marpit presentation"
    Use command: Markdown: Open Preview to the side 
    and not the command that belong to **Markdown Preview Enhanced**
     
![](images/markdown_open_preview.png)

### Toggle marpit
![](images/marpit_command.png)



![](images/marpit_toggle.png)

---

## Export
marp export to: HTML, PDF, PPTX

![](images/marpit_export.png)


```json
"markdown.marp.exportType": "pdf"
```

---
## How to and Tips
### theme

```yaml
---
title: demo marpit presentation
description: example slide
marp: true
size: 4:3
paginate: true
theme: gaia
---
```

![](images/builtin_theme.png)

### settings
- **paginate**: Shows page number on the slide or not
- **size**: 

### Images
```
---

![bg left](https://picsum.photos/720?image=29) 
- image on the left, with text on the right
---

![bg left:33%](https://picsum.photos/720?image=29) 
- same of above, but image is 33% of the slide)

---
```


![](images/marpit_images.png)


---

## Reference
- [Unleash Your Creativity with Marp Presentation Customization ](https://dev.to/chris_ayers/unleash-your-creativity-with-marp-presentation-customization-1cpn)