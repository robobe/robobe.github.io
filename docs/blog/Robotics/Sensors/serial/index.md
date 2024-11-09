---
tags:
    - serial
    - 
---

!!! tip check serial  port 
     
    check serial port using socat

    ```
    sudo apt install socat
    ```

    ```
    socat -d -d STDIN file:/dev/ttyUSB0,raw,echo=0
    ```


    ```bash
    2024/11/06 05:28:52 socat[503704] N using stdin for reading and writing
    2024/11/06 05:28:52 socat[503704] N opening character device "/dev/ttyUSB0" for reading and writing
    2024/11/06 05:28:52 socat[503704] N starting data transfer loop with FDs [0,0] and [5,5]
    echo
    echo
    ```