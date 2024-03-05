---
tags:
    - git
    - branch
    - tag
    
---
Git create local branch from remote branch and tags

## remote branch
```bash
# sync with remote
git fetch origin
```

```bash title="list all branch"
git branch -v -a
```

```bash
git switch -c <local branch name> <remote branch name without the remote prefix>

```

## tag

```bash title="sync with remote"
git fetch --all --tags
```

```bash title="list all remote tags"
git ls-remote --tags
```

```bash
git checkout tags/tag_name -b branch_name
```