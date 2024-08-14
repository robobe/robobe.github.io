---
tags:
    - nx
    - docker
    - toolkit
    - container
---

## install docker
- Jetpack 6.0
- Jetson Orin NX

```bash
sudo apt install apt-transport-https ca-certificates curl gnupg lsb-release

# gpg
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# add repository
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# install
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io
```

---

## Reference
- [A Beginner’s Guide to NVIDIA Container Toolkit on Docker](https://medium.com/@u.mele.coding/a-beginners-guide-to-nvidia-container-toolkit-on-docker-92b645f92006)