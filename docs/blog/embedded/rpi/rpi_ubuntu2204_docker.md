---
tags:
    - rpi
    - docker
    - ubuntu
---

# Install docker on RPi

install docker on rpi4 running ubuntu 22.04


```bash
 sudo apt-get update

```

```bash
 sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

```

```bash
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

```

```bash
 echo \
  "deb [arch=arm64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

```

```bash
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io

```

```bash title="set permission"
sudo usermod -aG docker $USER

```

---

## Reference
- [Docker on Ubuntu 20.04 Raspberry Pi 4 ](https://omar2cloud.github.io/rasp/rpidock/)