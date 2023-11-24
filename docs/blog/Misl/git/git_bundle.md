---
tags:
    - git
    - bundle
    - backup
---
# Git bundle

Git bundle is a utility that allow you to pack repository, branch, specific commit to signal file and expend them easily.

## Demo
- Share commits between two repositories that has the same **history**

```bash title="create repo with two commits"
# create folder and init git
mkdir repo1
git init

# Add commit on file test.txt
echo line1 > test.txt
git add .
git commit -m "line1"

# Add another commit on file test.txt
echo line2 >> test.txt
git add .
git commit -m "line2"

# show history
git --no-pager log --oneline  
```

```bash

# git bundle create <bundle name> <commot1> <commot2> <branch>
git bundle create two_commits.bundle 8aab30e 78d350b master
```

### use bundle
```bash title="create new repo and unbundle
mkdir repo2
git init

# unbundle to new branch
## unbundle to new branch, new branch should be merge and delete
git fetch two_commits.bundle master:feature

```

#### check if bundle can apply to repository
```bash
git bundle verify <bundle name> 

```

---

## Reference
- [git bundle](https://youtu.be/AVYlKbbA77M)