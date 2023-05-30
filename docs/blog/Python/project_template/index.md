---
description: Python project template
date: "2022-24-05"
banner: images/python.png
tags:
    - python
    - project
---
# Python project template

This post explain project structure and how to create github template repository

Other post show how to config virtual environment install package unittest and tools usage

- part 1: this post
- part 2: [venv and pip](venv_pip_and_requirements)
- part 3: [unitest using pytest](project_unittest)
- Tools:
    * [pylint](pylint.md)
    * [flake8](flake8.md)
    * [mypy](mypy.md)
    * [config isort, black, mypy and pylint with pre-commit hook](pre_commit.md)
- Demo: [Crate project](use_case.md)
- [Add projects documentation](docs.md)






## project tree
```
py_template/
├── .vscode
├── docs
├── Makefile
├── mkdocs.yml
├── pyproject.toml
├── py_template
├── README.md
├── .pylintrc
├── setup.cfg
├── setup.py
├── tests
|    └── requirements.txt
├── venv
└── version.py

```

| file name  | description  |
|---|---|
| Makefile  | automate validation and running tests  |
| .pylintrc | pylint rules |

---

## GitHUB
Set project as a Template
![](images/github_template.png)


[my python projct template on github](https://github.com/robobe/py_template)


---


# Reference
- [Modern python developers toolkit](https://pycon.switowski.com/0-introduction/)
- [template example](https://github.com/overfitted-cat/from-pybase)
- [Python Dev Environment](https://dev.to/bowmanjd/python-dev-environment-part-3-dependencies-with-installrequires-and-requirements-txt-kk3)
