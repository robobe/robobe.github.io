---
tags: 
    - yaml
    - yml
    - python
    - tutorial
---

# YAML: Yet Another Markup Language
Or `ain't markup language`

YAML is a digestible data serialization language often used to create configuration files with any programming language.
The basic structure of a YAML file is a map. `key: value`
YAML use **two spaces** per level of indentation

minimal yaml file

```yaml
version: 2
```

## Demo: Read yaml file with python

!!! tip "PyYAML"
    ```
    pip install PyYAML
    ```
     
```python
import yaml
from pathlib import Path
import json

p = Path(__file__).with_name("data.yaml")
with p.open("r") as f:
    data = yaml.load(f, Loader=yaml.FullLoader)

# Print pretty
print(json.dumps(data, indent=4))
```

## VSCode
[YAML Red Hat](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-yaml)

```
ext install redhat.vscode-yaml
```

## YAML Comments

```yaml
# comments: 2
```

## YAML Strings

```yaml
team: 76ers
team1: "76ers"
```

```json title="python parse result"
{
    "team": "76ers",
    "team1": "76ers"
}
```

## YAML Numbers

```yaml
n1: 1000
n2: 10.100
```

```json title="python parse result"
{
    "n1": 1000,
    "n2": 10.1
}
```

## YAML Boolean

```yaml
bool: true
bool1: on
bool2: off
bool11: yes
bool22: no
```

```json title="python parse result"
{
    "bool": true,
    "bool1": true,
    "bool2": false,
    "bool11": true,
    "bool22": false
}
```

## YAML Multi line Strings

```yaml
# Combine the multi line to one line in the output
multiple_line: >
  line 1
  line 2
  line 2

# Add new line for each 
multiple_line1: |
  line 1
  line 2
  line 2
```

```json title="python parse result"
{
    "multiple_line": "line 1 line 2 line 2\n",
    "multiple_line1": "line 1\nline 2\nline 2\n"
}
```

## YAML List

```yaml
data_list:
  - item1
  - item2

data_list1: [item1, item2]
```

```json title="python parse result"
{
    "data_list": [
        "item1",
        "item2"
    ],
    "data_list1": [
        "item1",
        "item2"
    ]
}
```

## YAML Dictionary

```yaml
dic:
  item1: 10
  item2: hello
```

```json title="python parse result"
{
    "dic": {
        "item1": 10,
        "item2": "hello"
    }
}
```

## YAML Complex type

```yaml title="list of objects"
objects:
  - name: name1
    number: 1
    is_bool: true
  - name: name2
    number: 3
    is_bool: false
```

```json title="python parse result"
{
    "objects": [
        {
            "name": "name1",
            "number": 1,
            "is_bool": true
        },
        {
            "name": "name2",
            "number": 3,
            "is_bool": false
        }
    ]
}
```

---

## Resources
- [Learn YAML in 10 Minutes](https://youtu.be/BEki_rsWu4E)