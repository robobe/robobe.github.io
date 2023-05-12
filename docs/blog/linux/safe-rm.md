---
tags:
    - linux
    - rm
    - ubuntu
---

# Safe-rm
safe-rm prevents the accidental deletion of important files by replacing rm with a wrapper
which checks the given arguments against a configurable blacklist of files and directories
which should never be removed.

```
sudo apt install safe-rm
```

## config
### path
- create soft link between safe-rm to `rm` in folder according to path priority

```bash
# check rm location
which rm

# check safe-rm location
which safe-rm

# check path folder order
echo $PATH

# crate soft link in the right location

# set at user level
ln -s /usr/bin/safe-rm /home/user/.local/bin/rm
```
  
### blacklist
```bash
# system level
/etc/safe-rm.conf
# user level
~/.config/safe-rm
```


# reference
- [safe-rm](https://manpages.ubuntu.com/manpages/focal/man1/safe-rm.1.html)
- [trash-cli](https://manpages.ubuntu.com/manpages/jammy/man1/trash.1.html)