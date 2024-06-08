---
tags:
    - git
    - hooks
    - pre-commit
---
# git pre-commit


All Git **hooks** are ordinary scripts that Git executes when certain events occur in the repository
like:
- pre-commit
- post-commit
- update
- post-merge
- pre-push
- [full list and more](https://githooks.com/)

git hooks locate in `.git/hooks` folder


# pre-commit

`pre-commit` A framework for managing and maintaining multi-language **pre-commit hooks**.


## Install
```bash
python -m pip install pre-commit
#or from pyz
# Download from release
python pre-commit-#.#.#.pyz
```


### usage
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

---

## Custom script demo

run shell script

```yaml
repos:
- repo: local
  hooks:
    - id: shell_script
      name: shell_script
      entry: path_to_shell_script.sh
      language: script
```

---

## References
- [pre-commit](https://pre-commit.com/)
- [4 Tools to Format & Check your Code with Pre-Commit](https://youtu.be/g82VVb8xbAU)
