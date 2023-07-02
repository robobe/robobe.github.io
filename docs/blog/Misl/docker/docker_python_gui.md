---
tags:   
    - docker
    - python
    - gui
    - app
---

# Docker and Python GUI application
Package and Deploy Python TKInter GUI application using Docker


## Project

```bash
.
├── .devcontainer
│   └── Dockerfile
├── py_gui_app
│   ├── app2.py
│   └── app.py
└── README.md
```

### Dockerfile
Using multistage Docker file
- Base
- Dev (TODO)
- Deploy


```Dockerfile
FROM python:3.8.12-slim as app_base

RUN apt-get update -y

# Install Tkinter
RUN apt-get install tk -y

# Deploy
FROM app_base as app

# Copy app.py script to docker root
ADD /py_gui_app/app.py .
# cmd use as arguments to entrypoint
CMD ["app.py"]  
ENTRYPOINT ["python3"]
```

- **RUN**: Execute commands and create new image layer.
- **CMD**: Sets the command and its arguments to executed after container start. It's can be override by docker `run` command  line arguments
- **ENTRYPOINT**: Configures the command to run when the container starts,

!!! tip CMD formats
    1. **Exec command**: `CMD ["executable", "param1", "param2"]`
    2. `CMD ["param1", "param2"]` is use with `ENTRYPOINT` and provide extra parameters
    3. **Shell format**: `CMD command param1 param2`
     

!!! tip ENTRYPOINT override
    ```
    docker run --entrypoint="path/to/custom/entrypoint" imagename
    ```

---

### TKInter

```python
import tkinter as tk

# Tkinter Window
root_window = tk.Tk()

# Window Settings
root_window.title('Application Title')
root_window.geometry('300x100')
root_window.configure(background = '#353535')

# Text
tk.Label(root_window, text='Hello World', fg='White', bg='#353535').pack()

# Exit Button
tk.Button(root_window, text='Exit', width=10, command=root_window.destroy).pack()

# Main loop
root_window.mainloop()
```

---


## Build And Usage
### Build app target

```bash
docker build --target app \
-f Dockerfile \
-t my_gui_app:app ..
```

|   Tag        |    description                       |
| ------------ | ------------------------------------ |
| --target app | Build docker using the **app** stage |
| -f           | Dockerfile name to use               |
| -t           | Tag image name                       |
| ..           | Set build context                    |


!!! note "Docker build context"
    A build's context is the set of files located in the specified PATH or URL . The `build` process can refer to any of the files in the context

     
### Run

```bash
docker run --entrypoint="/bin/sh" \
        -u=$(id -u $USER):$(id -g $USER) \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --rm \
        my_gui_app:app
```

![](images/run_python_gui_over_docker.png)


#### override entrypoint
- Login in to `sh` terminal 


```bash linenums="1" hl_lines="2"
docker run 
    --entrypoint="/bin/sh" \
    -u=$(id -u $USER):$(id -g $USER) \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --rm \
    -it \
    my_gui_app:app
```

---

## Reference
- [docker run vs cmd vs entry point](https://tonylixu.medium.com/docker-run-vs-cmd-vs-entrypoint-57f248b95889)
- [Dockerize python app](https://youtu.be/0UG2x2iWerk)