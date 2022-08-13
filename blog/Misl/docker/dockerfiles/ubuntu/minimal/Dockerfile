FROM ubuntu:20.04 as ubuntu_base

ENV LANG en_US.UTF-8
ARG DEBIAN_FRONTEND=noninteractive

# OS update, then clean up
RUN apt -yq update && \
    apt -yq upgrade && \
    apt -yq --no-install-recommends install \
        locales \
        locales-all \
        iproute2 \
        openssh-server && \
    apt clean && \
    rm -rf /tmp/* /var/tmp/* /var/lib/apt/archive/* /var/lib/apt/lists/*

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  #Add sudo support for the non-root user
  && apt update \
  && apt install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* 

WORKDIR /home/user