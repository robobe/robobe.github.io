---
tags:
    - git
    - submodules
---
# Submodules
Submodules are repositories inside other Git repositories.




```bash
git submodule add -b master [URL to Git repo] 
git submodule init

git submodule update

```

All submodules handle `.gitmodules` file


## Tips
submodule don't fetch from server automatically
to download all submodules

```bash title="fetch recursive"
git submodule update --init --recursive
```

---

## Reference
- [Handle Git Submodules with ease](https://miroslav-slapka.medium.com/handle-git-submodules-with-ease-55621afdb7bb)