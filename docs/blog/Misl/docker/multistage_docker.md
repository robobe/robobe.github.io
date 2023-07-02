---
tags:
    - docker
    - multi-stage
    - stage
---
# Docker multi-stage
With multi-stage builds, you use multiple FROM statements in your Dockerfile. Each FROM instruction can use a different base, and each of them begins a new stage of the build. You can selectively copy artifacts from one stage to another, leaving behind everything you don’t want in the final image

---

## Build stage 

```bash
docker build --target builder -f Dockerfile -t react-b:latest . 

# The --target the option allows you to specify a specific stage at which you'd like to stop.
```

---

## Reference
- [Building Images Faster and Better With Multi-Stage Builds](https://gcore.com/learning/speed-up-docker-builds-with-multi-stage/)