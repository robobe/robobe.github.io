---
tags:
    - nano
    - jetson
    - i2c
    - adafruit
    - 
---

# Connect AdaFruit 8*8 matrix to jetson nano


## Wiring

| Matrix | Nano         |
| ------ | ------------ |
| VCC    | 5VDC (2)     |
| GND    | GND (39)     |
| SDA    | I2C0_SDA(27) |
| SCL    | I2C0_CLK(28) |

## Check

```
i2cdetect -y -r 0
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: 70 -- -- -- -- -- -- --
```

## Python

```
pip install adafruit-circuitpython-ht16k33
```

```python title="ada.py" linenums="1" hl_lines="9 15"
import time
import busio
from adafruit_ht16k33 import matrix

# Create the I2C interface.
def handler(self, scl, sda, frequency):
    from adafruit_blinka.microcontroller.generic_linux.i2c import I2C as _I2C
    portId = 0
    self._i2c = _I2C(portId, mode=_I2C.MASTER, baudrate=None)

busio.I2C.init = handler
i2c = busio.I2C(-1, -1)

# creates a 8x8 matrix:
matrix = matrix.Matrix8x8(i2c)

# edges of an 8x8 matrix
col_max = 8
row_max = 8

# Clear the matrix.
matrix.fill(0)
col = 0
row = 0

while True:

    # illuminate a column one LED at a time
    while col < col_max:
        matrix[row, col] = 2
        col += 1
        time.sleep(.2)

    # next row when previous column is full
    if row < row_max:
        row += 1
        col = 0

    # clear matrix, start over
    else:
        row = col = 0
        matrix.fill(0)
```

### Monkey patch

The original package not work with `Jetson nano` and linux board at all
The detector math general linux board but not initialize the i2c correct

The MonkeyPatch override the init process and override the `i2c` object creation

line 15: The `busio.I2C(-1, -1)` init SDA, SCL pin that not use in linux board

!!! note "gpio"
    Add user to `GPIO` group

    ```bash
    sudo usermod -aG gpio $USER
    ```
     
