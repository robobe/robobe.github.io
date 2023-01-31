---
title: git bundle
tags:
    - git
    - bundle
---
Git bundle is a utility that allow you to pack repository, branch, specific commit to signal file and expend them easily.

## Demo: Bundle Range of commits
### bundle
```bash title="view commits"
git log --oneline origin/master..master
```

```bash title="bundle commits"
git bundle create patch.bundle origin/master..master
```

### Unbundling

```bash title=""
git branch temp

git switch temp

git fetch -u patch.bundle master:temp

git switch master

git merge temp

git branch --delete temp
```

---

# Reference
- [A Guide for Bundling Git Repos](https://initialcommit.com/blog/git-bundle)