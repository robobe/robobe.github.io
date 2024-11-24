---
tags:
    - vscode
    - ros2
    - shell
    - tips
---
# Shell and Dev environment setup

## Shell environment

### Prompt
[How To Change Your PS1 Bash Prompt (And Add Emojis)](https://tynick.com/blog/06-12-2019/how-to-change-your-ps1-bash-prompt-and-add-emojis/)
```bash
#/bin/sh

source /opt/ros/humble/setup.bash

# prompt
function parse_git_dirty {
  [[ $(git status --porcelain 2> /dev/null) ]] && echo "*"
}
function parse_git_branch {
  git branch --no-color 2> /dev/null | sed -e '/^[^*]/d' -e "s/* \(.*\)/ (\1$(parse_git_dirty))/"
}

export PS1="\n\H \[\033[32m\]\w\[\033[33m\]\$(parse_git_branch)\[\033[00m\] 🐢 $ "

# ros log settings
RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {line_number})"

```


# VSCode

- [Variables Reference](https://code.visualstudio.com/docs/editor/variables-reference)
