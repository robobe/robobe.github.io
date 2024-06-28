---
tags:
    - tips
    - linux
    - ubuntu
    - shell
---

# Tips

## Port usage by process

```bash
sudo lsof -i TCP:22
```

##

---

## Clipboard
Using clipboard from cli

```bash
ls -l | xclip -sel clip
```

```bash
alias copy="xclip -sel clip"

#usage
ls | copy
```