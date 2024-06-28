---
tags:
    - vscode
    - devcontainer
    - server
    - docker
    - volume
---

# Offline dev container

## backup vscode volume
VSCode save devcontainer server side in docker volume called `/vscode`

```
docker volume ls
```

We can backup this volume to tar and untar it in the offline
- Run docker that use the volume


```bash title="backup"
docker run --rm --volumes-from <container id> \
-v $(pwd):/backup \
busybox tar cvf /backup/vscode.tar /vscode
```

```bash title="restore"
docker run --rm --volumes-from <container> -v $(pwd):/backup busybox sh -c "cd /vscode && tar xvf /backup/backup.tar --strip 1"
```

!!! note 
    **--volumes-from**  links volumes between running container


---

## Reference
- [A Detailed Guide on Docker Volumes](https://refine.dev/blog/docker-volumes/#steps-to-restore-a-docker-volume)