---
title: Python project template 
description: tests using pytest
date: "2022-24-05"
banner: images/python.png
tags:
    - python
    - project
    - template
    - unittest
    - pytest
---

## Install and Setup
- Add `pytest` to `requirements-dev.txt`  
- Run `pip install -r requirements-dev.txt` again
- Create `tests` sub folder under root project folder
- Add tests modules files prefix with `test_`
- Add function to modules start also with `test_` prefix
- Install project using `pip install -e .`
- Run `pytest`

## demo

```
demo_proj/
├── .vscode
├── src
|    └── demo.py
├── setup.cfg
├── setup.py
├── tests
|    └── test_demo.py
├── venv
├── requirements.txt
├── requirements-dev.txt
└── version.py

```
---

## VSCode
### Test Explorer UI
![](images/test_explorer.png)
Run your tests in the Sidebar of Visual Studio Code

### Test Adapter Converter
![https://marketplace.visualstudio.com/items?itemName=ms-vscode.test-adapter-converter](images/test_adapter.png)
Converter extension from the Test Adapter UI to native VS Code testing
