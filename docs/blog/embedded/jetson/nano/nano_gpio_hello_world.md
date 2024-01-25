---
tags:
    - jetson
    - nano
    - gpio
    - embedded
---

![](images/nano_gpio_header.png)

![](images/nano_gpio_pinout.png)

nano gpio: 3.3v
nano has no pullup / pulldown 



## gpiod

```bash title=install
sudo apt install gpiod
sudo apt install libgpiod-dev
```

### Change permission
#### Group
- Check for `gpio` group
- Add user to `gpio` group

```bash
sudo groupadd -f -r gpio
sudo usermod -a -G gpio your_user_name
```

#### udev
- Add file to `/etc/udev/rule.d` folder

!!! note "rules"
    The rule are generic for jetson and rpi
    rules are board related
     
```bash title="99-gpio.rules"
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", \
    PROGRAM="/bin/sh -c 'chown root:gpio /dev/%k; chmod 660 /dev/%k'"

SUBSYSTEM=="pwm", KERNEL=="pwmchip*", ACTION=="add", \
    PROGRAM="/bin/bash -c 'chown root:gpio /sys%p/{,un}export; chmod 220 /sys%p/{,un}export'"
SUBSYSTEM=="pwm", KERNEL=="pwmchip*", ACTION=="change", \
    PROGRAM="/bin/bash -c 'chown root:gpio /sys%p/%E{EXPORT}/{period,duty_cycle,enable}; chmod 660 /sys%p/%E{EXPORT}/{period,duty_cycle,enable}'"

SUBSYSTEM=="spidev", KERNEL=="spidev*", ACTION=="add", \
    GROUP="gpio", MODE="0660"

```


```bash title="check rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

```bash
# Before udev rules
ll /dev/gpiochip0
crw------- 1 root root 254, 0 Jan 20 23:47 /dev/gpiochip0

# after reboot
ll /dev/gpiochip0
crw-rw---- 1 root gpio 254, 0 Jan 20 23:49 /dev/gpiochip0
```
---

## bash
- Read gpio 12

```bash
gpioinfo gpiochip0 | grep 12
#
line  12:  "SPI1_MOSI"       unused   input  active-high
```


```bash
# short to ground
gpioget gpiochip0 12
0

# short to 3.3 line
gpioget gpiochip0 12
1

```

## python

!!! warning "python version"
    Hardware: Jetson NANO
    OS: Ubuntu 18.04

    Install python3.8 for working with `gpiod` package

    gpiod version: 1.5.4

    ```
    sudo apt install python3.8
    ```
     
```
pip install gpiod
```

```python title="simple get"
import gpiod
import time

GPIO_L = 12

chip = gpiod.chip("gpiochip0")
line = chip.get_line(GPIO_L)

line_config = gpiod.line_request()
line_config.consumer ="me"
line_config.request_type = gpiod.line_request.DIRECTION_INPUT
line.request(line_config)

while True:
    v = line.get_value()
    print(v)
    time.sleep(1)
```

```bash
sudo gpioinfo gpiochip0 | grep 12
line  12:  "SPI1_MOSI"         "me"   input  active-high [used]

```

### Output

![](images/jetson_nano_gpio_led.drawio.png)

```python title="simple set"
import gpiod
import time

GPIO_L = 12

chip = gpiod.chip("gpiochip0")
line = chip.get_line(GPIO_L)

line_config = gpiod.line_request()
line_config.consumer ="me"
line_config.request_type = gpiod.line_request.DIRECTION_OUTPUT
line.request(line_config)

while True:
    line.set_value(1)
    time.sleep(1)
    line.set_value(0)
    time.sleep(1)
```