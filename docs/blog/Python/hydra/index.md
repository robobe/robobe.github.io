---
tags:
    - python
    - hydra
    - yaml
    - configuration
---

# Hydra
Hydra ...   
The key feature is the ability to dynamically create a hierarchical configuration by composition and override it through **config files** and the **command line**.

## Demo
- Load config from `yaml` file
- Convert yaml config to typed object using dataclass
- Override `yaml` data from CLI

```yaml title="config.yaml" linenums="1" hl_lines="1-3"
hydra:  
  job:
    chdir: True # (1)

db:
  driver: mysql
  table: bar
  user: bar
  password: foo
```

1. disable hydra output warning


```python title="map yaml file to typed object using dataclass"
@dataclass
class DB:
    driver: str
    table: str
    user: str
    password: str


@dataclass
class Settings:
    db: DB


cs = ConfigStore.instance()
# name `base_config` is used for matching it with the main.yaml's default section
cs.store(name="base_config", node=Settings)
```


```python title="full example"
import cv2
import hydra
# from omegaconf import DictConfig, OmegaConf
from hydra.core.config_store import ConfigStore
from dataclasses import dataclass


@dataclass
class DB:
    driver: str
    table: str
    user: str
    password: str


@dataclass
class Settings:
    db: DB


cs = ConfigStore.instance()
# name `base_config` is used for matching it with the main.yaml's default section
cs.store(name="base_config", node=Settings)


@hydra.main(config_path=".", config_name="config", version_base="1.1")
def my_app(cfg: Settings) -> None:
    # print(OmegaConf.to_yaml(cfg))
    print(cfg)
    print(type(cfg))
    print(cfg.db.driver)
    print(cfg.db.table)

if __name__ == "__main__":
    my_app()

```

### usage
```
```
---

!!! note "Disabled output folder"

    ```yaml
    defaults:  
      - _self_  
      - override hydra/hydra_logging: disabled  
      - override hydra/job_logging: disabled  
      
    hydra:  
      output_subdir: null  
      run:  
        dir: .
    ```