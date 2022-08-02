---
title: Hello docker - Network
tags:
    - docker
    - tutorial
---

## Assign port 

```
-p HostPort:ContainerPort
```
### demo
- Run webserver and map internal port to port 8080 on host


```bash
# run docker container as daemon (-d)
# assing container name (--name)
# map ports (-p)
docker run -d --name MyWebServer -p 8080:80 httpd
# check port mapping
docker port MyWebServer 
80/tcp -> 0.0.0.0:8080
80/tcp -> :::8080

```

---

# Reference
- [Docker Tutorial Series](https://rominirani.com/docker-tutorial-series-a7e6ff90a023)