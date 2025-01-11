---
tags:
    - docker
    - nvidia
    - container-toolkit
---
# NVIDIA docker
NVidia container toolkit enables GPU acceleration in docker container

- Install Toolkit
- Configure Docker
- Run with GPU
  
## Install


```bash title="add nvidia toolkit repository"
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

```bash title="install"
sudo apt update
sudo apt-get install -y nvidia-container-toolkit
```

---

## Configure Docker

```
sudo nvidia-ctk runtime configure --runtime=docker
```

```
systemctl --user restart docker
```

---

## Usage

run image form docker hub with cuda 

```
docker pull nvidia/cuda:12.2.2-devel-ubuntu22.04
```

!!! top ""
     Run container with `--gps all ` to enable nvidia gpu in container

### Run
```bash
docker run --gpus all -it nvidia/cuda:12.2.2-devel-ubuntu22.04
```


### check
```bash 
nvidia-smi
nvcc --version
```

---

## Reference
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)