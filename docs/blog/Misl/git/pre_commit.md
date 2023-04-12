---
tags:
    - git
    - hooks
    - pre-commit
---
# git pre-commit
All Git **hooks** are ordinary scripts that Git executes when certain events occur in the repository

git hooks locate in `.git/hooks` folder

`pre-commit` is a library that help us manage the `pre-commit` hook


## Install
```bash
python -m pip install pre-commit
#or from pyz
# Download from release
python pre-commit-#.#.#.pyz
```

## pre-commit
pre-commit is a tool that help us config and install git hook for pre-commit event.
### usage
- Install
- Create config file at project root folder
- Add hooks
- Install hooks
- Test / Run hooks

#### config hooks
```
touch .pre-commit-config.yaml
```

```yaml
repos:
-   repo: https://github.com/psf/black
    rev: 23.3.0
    hooks:
    -   id: black

```
!!! note "supported hooks"
    `rev` can take from github project releases page

!!! note "supported hooks"
     Get hooks list from [pre-commit](https://pre-commit.com/hooks.html)

#### install hook
```
pre-commit install
```

#### Test/ run
```bash
#-a all-files
pre-commit run -a

```

### Tips
Action order
1. config modify action hooks
2. config check action hooks 

## References
- [pre-commit](https://pre-commit.com/)
- [4 Tools to Format & Check your Code with Pre-Commit](https://youtu.be/g82VVb8xbAU)
