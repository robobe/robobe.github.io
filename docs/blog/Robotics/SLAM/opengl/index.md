---
tags:
    - opengl
    - python
    - pangolin
---


## OpenGL
OpenGL (Open Graphics Library) is a cross-platform, open-source application programming interface (API) used for rendering 2D and 3D vector graphics.

### install

```bash
pip install PyOpenGL
pip install PyOpenGL_accelerate
```

### Hello

OpenGL with pygame

```bash
pip install pygame
```

!!! note "glfw"
     `GLFW` Graphics Library Framework is a library design for creating windows handling inputs and managing OpenGL or Vulkan context

    ```
    pip install glfw
    ```

     ```python
    import glfw
    from OpenGL.GL import *

    # Initialize the library
    if not glfw.init():
        raise Exception("GLFW can't be initialized")

    # Create a windowed mode window and its OpenGL context
    window = glfw.create_window(640, 480, "Hello World", None, None)
    if not window:
        glfw.terminate()
        raise Exception("GLFW window can't be created")

    # Make the window's context current
    glfw.make_context_current(window)

    # Main loop
    while not glfw.window_should_close(window):
        # Render here, e.g. using OpenGL
        glClear(GL_COLOR_BUFFER_BIT)

        # Swap front and back buffers
        glfw.swap_buffers(window)

        # Poll for and process events
        glfw.poll_events()

    # Clean up and terminate the library
    glfw.terminate()

     ```


```python
import pygame as pg
from OpenGL.GL import *

class App():
    def __init__(self):
        pg.init()
        self.screen = pg.display.set_mode((640, 480), pg.OPENGL | pg.DOUBLEBUF)
        self.clock = pg.time.Clock()
        glClearColor(1.0, 0.0, 0.0, 1.0)
        self.running = True

    def run(self):
        while self.running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    self.running = False
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            pg.display.flip()
            self.clock.tick(60)
        self.quit()

    def quit(self):
        pg.quit()

if __name__ == "__main__":
    app = App()
    app.run()
```

---

## Pangolin
Pangolin is a lightweight portable rapid development library for managing OpenGL display / interaction and abstracting video input.
it has a flexible real-time plotter for visualing graphical data

!!! note ""
     There are other github pangolin repository
     I success build this repository with it's python bindings

     the python import `pypangolin`

     ```
     import pypangolin
     ```


```bash
git clone https://github.com/stevenlovegrove/Pangolin 
cd Pangolin
git checkout v0.9.1
mkdir build
cd build 
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14 -DCMAKE_INSTALL_PREFIX=/usr/local .. 
make -j8 
make install
```

```bash
set PYTHONPATH

PYTHONPATH=/Pangolin/build/pypangolin-0.9.1.data/purelib

#

python

import pypangolin
```