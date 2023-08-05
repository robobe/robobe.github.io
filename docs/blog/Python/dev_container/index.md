---
tags:
    - python
    - vscode
    - dev container
    - docker
---

# Python project using VSCode devcontainer
Create python project using devcontainer


```bash title="basic project"
├── .devcontainer
│   ├── devcontainer.json
│   ├── docker-compose.yaml
│   ├── Dockerfile
│   └── postCreateCommand.sh
├── .dockerignore
├── .flake8
├── .gitignore
├── py_devcontainer_demo
│   ├── __init__.py
│   ├── main.py
│   └── utils.py
├── pyproject.toml
├── README.md
├── requirements-dev.txt
├── requirements.txt
├── scripts
│   └── clean.sh
├── setup.py
├── tests
│   ├── __init__.py
│   └── test_demo.py
└── .vscode
    ├── settings.json
    └── tasks.json

```

## Docker multistage
- Create multistage docker
    - base
    - dev
- Set devcontainer `target` attribute to requested target

```Dockerfile title="Dockerfile"
FROM ubuntu:22.04 as python_base
...
FROM python_base as development
...

```


```json title="devcontainer.json" linenums="1" hl_lines="6"
{
    "name": "python-dev",
    "build": {
        "dockerfile": "Dockerfile",
        "context": "..",
        "target": "development"
    }
}
```

!!! note "build multistage docker"
    ```bash
    docker build --target development
    ```
#### files     
- [Dockerfile](.devcontainer/Dockerfile)
- [devcontainer.json](.devcontainer/devcontainer.json)


---

## Permission
- Dockerfile: Add none root user (named user)
- Set devcontainer login as that user


```dockerfile title="Dockerfile"
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo tzdata \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/* 
```

```json title="devcontainer.json"
{
    "remoteUser": "user",
}
```

!!! note "remoteUser vs containerUser"
    - **remoteUser**: The user VS Code Server will be started with. The default is the same user as the container.
    - **containerUser**: The user the container will be started with. The default is the user on the Docker image.
     
---

## requirements
There at lest two option to install `requirement.txt`

- **Option 1**: Install `requirement.txt` using  `devcontainer.json` property like `postCreateCommand`

```json title="devcontainer.json"
{
    "postCreateCommand": "pip install -r /workspaces/py_docker_tutorial/requirements-dev.txt",
}
```

- **Option 2**: Install `requirements` as docker layer

```docker title="Dockerfile"
COPY ./requirements.txt /tmp/requirements.txt
COPY ./requirements-dev.txt /tmp/requirements-dev.txt
RUN pip install --no-`cache-dir --upgrade -r /home/user/requirements-dev.txt
```

---

## develop
- Register the project

```bash
pip install --prefix=~/.local -e .
```

---

## build wheel and deb

```dockerfile 
RUN apt-get update && \
    apt-get install -y \
        python3-all \
        debhelper \
        dh-python \
        python3-stdeb \
        locales \
        locales-all \
        && rm -rf /var/lib/apt/lists/*
```

### vscode tasks

```json title="tasks.json"
"tasks": [
        {
            "label": "build deb",
            "type": "shell",
            "command": "python3 setup.py --command-packages=stdeb.command bdist_deb",
            "problemMatcher": []
        },
        {
            "label": "build wheel",
            "type": "shell",
            "command": "python3 setup.py bdist_wheel",
            "problemMatcher": []
        },

    ]
```

!!! note "build output folder"
    - **deb**: deb_dist folder
    - **wheel**: dist folder
     




---


## Testing
- Install pytest (requirements-dev.txt)
- config vscode

```json title="devcontainer.json"
{
    "customizations": {
        "vscode": {
            "settings": {
                "python.testing.pytestArgs": [
                                "tests"
                            ],
                "python.testing.unittestEnabled": false,
                "python.testing.pytestEnabled": true
            }
        }
    }
}
```

---

## clean

- Clean Task

```json title=".vscode/tasks.json"
{
    "label": "clean",
    "type": "shell",
    "command": "/usr/bin/bash -x scripts/clean.sh",
    "problemMatcher": []
}
```

```bash title="scripts/clean.sh"
rm -rf deb_dist
rm -rf dist
find . | grep -E "(/__pycache__$|\.pyc$|\.pyo$)" | xargs rm -rf
```

!!! tip "run bash script without execute permission set"
    ```bash
    bash -x scripts/clean.sh
    ```

#### files
- [vscode tasks](.vscode/tasks.json)
- [script](scripts/clean.sh)
       
---

## python tools
- mypy
- formatter (black, isort)
- flake8 (linter)

---

## VSCode extensions
- Python
- Tabout
- Code Spell Checker
- Tasks

#### TabOut
Tab out quotes, brackets. etc
[marketplace](https://marketplace.visualstudio.com/items?itemName=albert.TabOut)

#### Code Spell Checker
Spelling checker for source code
[marketplace](https://marketplace.visualstudio.com/items?itemName=streetsidesoftware.code-spell-checker)
#### Tasks
Load VSCode Tasks into Status Bar.
[marketplace](https://marketplace.visualstudio.com/items?itemName=actboy168.tasks)

---

## GUI
### tk
- install python3-tk
- Add x11 support to docker execution

```dockerfile
RUN apt-get update && apt-get install -y \
    tk \
    python3-tk \
```

```json title="devcontainer.json"
{
    "runArgs": [
        "--volume=/tmp/.X11-unix:/tmp/.X11-unix"
    ],
    "containerEnv": {
        "DISPLAY": "${localEnv:DISPLAY}"
    },
}
```

```python title="main.py"
import tkinter as tk

# Tkinter Window
root_window = tk.Tk()

# Window Settings
root_window.title("Application Title")
root_window.geometry("300x100")
root_window.configure(background="#353535")

# Text
tk.Label(root_window, text="Hello World", fg="White", bg="#353535").pack()

# Exit Button
tk.Button(root_window, text="Exit", width=10, command=root_window.destroy).pack()

# Main loop
root_window.mainloop()

```

---
## TODO
#### docker-compose
- Add docker-compose to devcontainer.json
- Launch dev and other services


```json title="devcontainer.json"
"dockerComposeFile": "docker-compose.yaml",
"service": "app",
"workspaceFolder": "/workspace",
```

---

## References
- [devcontainer.json schema](https://containers.dev/implementors/json_schema/)
- [Variables Reference](https://code.visualstudio.com/docs/editor/variables-reference)
