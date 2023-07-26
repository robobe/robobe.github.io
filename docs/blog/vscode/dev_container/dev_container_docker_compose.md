---
tags:
    - dev container
    - vscode
    - docker
    - docker compose
---

```bash
├── docker-compose.yml
├── api
│   ├── api
│   │   └── main.py
│   └── .devcontainer
│       ├── devcontainer.json
│       ├── Dockerfile
│       └── requirements.txt
└── frontend
    ├── .devcontainer
    │   ├── devcontainer.json
    │   ├── Dockerfile
    │   └── requirements.txt
    └── frontend
```
## docker compose
```yaml
version: '3.4'

services:
  frontend:
    image: frontend
    tty: true
    build:
      context: ./frontend/.devcontainer
      dockerfile: ./Dockerfile
    volumes:
      - .:/workspace:cached
    depends_on:
      - api
    ports:
      - "8000:8000"
    networks:
      - myNetwork

  api:
    image: api
    tty: true
    build:
      context: ./api/.devcontainer
      dockerfile: ./Dockerfile
    volumes:
      - .:/workspace:cached
    networks:
      - myNetwork


networks:
  myNetwork:
    driver: bridge
```

---

## api

```json
{
    "name": "api",
    "dockerComposeFile": "../../docker-compose.yml",
    "workspaceFolder": "/workspace/api",
    "service": "api",
}
```

## Reference
- [Create a Multi-service Development Environment With VS Code and Docker](https://betterprogramming.pub/create-a-multi-service-development-environment-with-vs-code-and-docker-e58b2b611278)
- [Dev Container metadata reference](https://containers.dev/implementors/json_reference/)
- [Use Docker Compose](https://code.visualstudio.com/docs/containers/docker-compose)