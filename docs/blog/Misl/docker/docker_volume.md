---
tags:
    - docker
    - volume
    - mount
    - compose
---

# Docker Volume
The purpose of using Docker volumes is to persist data outside the container so it can be backed up or shared.

## bind mount

```bash title="syntax"
-v <source>:<destination>:<options>
```

```bash title="simple"
docker run -it \
    --rm \
    --name dev \
    -v /home/user/tmp:/app \
    ubuntu:22.04 /bin/bash
```

```bash title="mount as readonly"
docker run -it \
    --rm \
    --name dev \
    -v /home/user/tmp:/app:ro \
    ubuntu:22.04 /bin/bash
```

---

## volume mount
### unnamed volume
```bash title="mount volume"
docker run -it --rm \
    --name dev \
    -v /app \
    ubuntu:22.04 /bin/bash 
```

```bash
docker volume ls
#
docker volume ls
DRIVER    VOLUME NAME
local     88a466302764f0693e60746c1f75d2b699d432bbf47de2bdc1093af49596bbba

```

### named volume

```bash title="syntax"
-v <name>:<destination>:<options>
```

```bash title="named volume"
docker run -it --rm \
--name dev \
-v my_volume:/volume \
ubuntu:22.04 /bin/bash
```

```bash title="docker volume"
docker volume ls
DRIVER    VOLUME NAME
local     my_volume

```

!!! note "Docker volumes location"
     ```bash
     /var/lib/docker/volumes/
     ```

     demo: my_volume volume

    ```bash
    docker volume inspect my_volume
    [
        {
            "CreatedAt": "2023-08-10T19:57:55+03:00",
            "Driver": "local",
            "Labels": null,
            "Mountpoint": "/var/lib/docker/volumes/my_volume/_data",
            "Name": "my_volume",
            "Options": null,
            "Scope": "local"
        }
    ]

    ```

---

## Demo: volume and docker compose
- create volume: `my_volume`
- use volume from `docker-compose`

```bash
docker volume create my_volume
```

```yaml title="docker-compose up" linenums="1" hl_lines="15"
version: "3"

services:
  demo:
    container_name: my_container_name
    image: ubuntu:22.04
    hostname: dev
    stdin_open: true 
    tty: true
    volumes:
      - my_volume:/app

volumes:
  my_volume:
    external: true # (1)
```

1. Declare `my_volume` as external value created by `docker volume create`


---

## Demo: Bind file

```bash
docker run -it \
    --rm \
    --name dev \
    -v $(pwd)/settings.yaml:/root/settings.yaml \
    ubuntu:22.04 \
    /bin/bash
```

!!! warning "source file"
    source file must have full path


---

## mount option
`mount` option is alternative to `-v` 
`mount` is more verbose and build from key, value pair

```bash
--mount type=[bind, volume, tmpfs],source=[volume_name],destination=[path_in_container], 
```
     
!!! note 
    if using `mount` option and mount not existing source, docker throw an error  
    in `volume` option docker well create a new source for us.

```bash
docker run -it --rm \
--name dev \
--mount type=bind,source=/home/user/tmp,destination=/app \
ubuntu:22.04 \
/bin/bash

```
     
```bash
docker inspect dev
#
# output only mount section
"Mounts": [
            {
                "Type": "bind",
                "Source": "/home/user/tmp",
                "Destination": "/app",
                "Mode": "",
                "RW": true,
                "Propagation": "rprivate"
            }
        ],

```

---

## Backup / Restore

```bash title="backup"
docker run --rm --volumes-from <container id> \
-v $(pwd):/backup \
busybox tar cvf /backup/vscode.tar <volume name>
```

```bash title="restore"
docker run --rm --volumes-from <container> \
-v $(pwd):/backup busybox \
sh -c "cd <volume name> \
&& tar xvf /backup/backup.tar --strip 1"
```