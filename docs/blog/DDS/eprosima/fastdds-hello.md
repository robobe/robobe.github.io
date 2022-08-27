---
title: eprosima fastdds hello
tags:
    - dds
    - eprosima
    - fastdds
---

## Objective
- Run fastdds basic example

## Setup
- Download eprosima fastdds docker [download](https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-dds/eprosima-fast-dds-2-7-1/ubuntu-fastdds-v2-7-1-tar?format=raw)
- Load docker 

```
docker load -i <path>/ubuntu-fastdds v2.7.1.tar
```

- Run docker

```
docker run -it --privileged ubuntu-fastdds:v2.7.1 
```

## Run Hello example
`Hello world` example locate at `/usr/local/eprosima/fastrtps/examples/cpp/dds/HelloWorldExample/bin`

- Run tmux start new session
- Split window 
- Run publisher in Pane1
- Run subscriber in Pane2

```bash title="tmux new session"
tmux new-session
#
# press c-b and " to split horizontal
```

```bash title="publisher"
# -s number of samples
# -i interval in ms
./DDSHelloWorldExample publisher -s 20 -i 500

Starting 
Publisher running 20 samples.
Publisher matched.
Message: HelloWorld with index: 1 SENT
Message: HelloWorld with index: 2 SENT
Message: HelloWorld with index: 3 SENT
Message: HelloWorld with index: 4 SENT
```

```bash title="subscriber"
./DDSHelloWorldExample subscriber
# 
Subscriber matched.
Message HelloWorld 1 RECEIVED
Message HelloWorld 2 RECEIVED
Message HelloWorld 3 RECEIVED
Message HelloWorld 4 RECEIVED

```

## Profiles

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/eprosima/fastrtps/examples/cpp/dds/HelloWorldExample/bin/DEFAULT_FASTRTPS_PROFILES.xml
```