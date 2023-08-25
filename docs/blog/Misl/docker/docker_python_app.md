---
tags:   
    - docker
    - python
    - app
    - devcontainer
---

# Docker Python application
Using `pysimplegui` to build simple python GUI application

Using Docker as production, development and test environment
Using VSCode devcontainer as development setup

## Dockerfile
Build from four stage:
1. **python_base**: base on ubuntu 22.04 that add user and install python
2. **deploy**: Add python application dependencies install by **apt** and **pip** (without the application it self)
3. **prod**: Install the application using whl build by **development**
4. **development**: Add all packages need by the development and build system

---

## Helper scripts
### Deploy
Check that the application can installed and run

- Deploy container include all package needed by the application for running (apt and pip)
- Run docker container with **user** permission and **x11** support
- Create share from `<project>`/dist folder to install the application on docker container
- Install run by the user from `/dist`  folder using `pip`


```bash title="run_deploy_container.sh"
docker run -it --rm \
    --name py_deploy \
    --hostname deploy \
    -u=$(id -u $USER):$(id -g $USER) \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/dist:/dist \
    py_gui_demo:deploy \
    /bin/bash
```

### Prod
Create docker container that run the application

- Base on **prod** stage that install all application package dependencies
- Get the application version to build from outside (vscode task, command line)
- Set `entrypoint`

!!! note "PATH"
    The application entry point installed in `<user home>/.local/bin`

    The docker add this environment variable with **ENV** command
    ```
    ENV PATH=${PATH}:/home/user/.local/bin
    ```
     
```Dockerfile title="production stage"
# ###################      production     #############################

FROM deploy as prod
ARG APP_VER=0
USER user
WORKDIR /home/user
RUN echo ${APP_VER}
COPY ./dist/py_gui_demo-${APP_VER}-py3-none-any.whl /tmp
RUN pip install /tmp/py_gui_demo-${APP_VER}-py3-none-any.whl
ENTRYPOINT ["my_app"]
```

```bash title="run_prod_container.sh"
docker run -it --rm \
    --name py_prod \
    --hostname prod \
    -u=$(id -u $USER):$(id -g $USER) \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    py_gui_demo:prod
```

### Base
Create docker container that include only the ubuntu base with python and pip install without all application package dependencies, use to check **whl** and **deb** installation 

- Container with user context and x11 support
- Share `dist` and `deb_dist` folder from host


```bash title="run_base_container.sh"
docker run -it --rm \
    --name py_base \
    --hostname base \
    -u=$(id -u $USER):$(id -g $USER) \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/dist:/dist \
    -v $(pwd)/deb_dist:/deb_dist \
    py_gui_demo:base \
    /bin/bash

```



---


## VSCode
### devcontainer

### tasks
- Add task build each dockerfile stage

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "build deploy container",
            "type": "shell",
            "command": "docker build --target deploy -t ${workspaceFolderBasename}:deploy -f .devcontainer/Dockerfile .",
            "problemMatcher": []
        },
        {
            "label": "build base container",
            "type": "shell",
            "command": "docker build --target python_base -t ${workspaceFolderBasename}:base -f .devcontainer/Dockerfile .",
            "problemMatcher": []
        },
        {
            "label": "build prod container",
            "type": "shell",
            "command": "docker build --target prod -t ${workspaceFolderBasename}:prod -f .devcontainer/Dockerfile --build-arg APP_VER=${input:app_version} .",
            "problemMatcher": []
        }

    ],
    "inputs": [
        {
            "id": "app_version",
            "description": "app_version",
            "options": ["0.0.1", "0.0.2"],
            "type": "pickString"
        },
    ]
}
```