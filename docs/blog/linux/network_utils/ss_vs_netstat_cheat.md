---
title: ss vs netstat command cheat sheet
tags:
    - network
    - ss
    - netstat
---

**Netstat** is a command-line network utility used to display network connections for the TCP/UDP and more.  
**ss** is a utility used to investigate sockets in Linux and Unix systems.

|                                               | ss  | netstat      |
| --------------------------------------------- | --- | ------------ |
| List all listening tcp ports and process name | ss -tlp   | netstat -ltp |
| List all listening udp ports and process name | ss -tun   | netstat -lup |
| filter by port number                         | ss -a dst :22 |  |

!!! note
     Run ss and netstat command with sudo to see process name for ports under 1024


