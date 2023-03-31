---
title: ROS2 project deploy from dev to production
tags:
    - ros2
    - projects
    - deploy
---

# Objective
- write basic python package
  - with external dependencies
- git hook
- run lint
- run tests
- pack the package
- deploy as `DEB` package
- check deploy package with docker
  - multi arch (x86, arm)

# steps
- [minimal python package with pub sub nodes](minimal_python_package.md)
- [create debian package](create_deb_package.md)
- [create docker image](make_docker.md)
- [run with docker compose](run_with_docker_compose.md)