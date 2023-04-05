---
tags:
    - python
    - click
    - cli
---

# Click

Click is a Python package for creating beautiful command line interfaces in a composable way with as little code as necessary. It’s the “Command Line Interface Creation Kit”. 
[click web site](https://click.palletsprojects.com)

## install
```bash
pip install click
```

## Demo
A function becomes a Click command line tool by decorating it through `click.command()`

```python
import click

@click.command()
def hello():
    print("hello")

if __name__ == "__main__":
    hello()
```

**usage**
```bash
python hello_click.py --help
Usage: hello_click.py [OPTIONS]

Options:
  --help  Show this message and exit.
```

---

### Add option
Option has a lot of options ...
[Options](https://click.palletsprojects.com/en/8.1.x/options/#options)

```python
import click

@click.command()
@click.option("--count", default=1, help="No. of hello's")
def hello(count=0):
    for _ in range(count):
        print("hello")

if __name__ == "__main__":
    hello()
```

!!! note "Function default value"
    Add default value in function just for lint , to disabled errors
    click option decorate is charge for default add pass the argument from command line

**usage**
```bash
python 
hello_click.py --help
Usage: hello_click.py [OPTIONS]

Options:
  --count INTEGER  No. of hello's
  --help           Show this message and exit.
```

```bash
python hello_click.py --count=3
```

---

### Add arguments
Arguments work similarly to options but are positional, they has less features and `click` will not create document for them 
[more](https://click.palletsprojects.com/en/8.1.x/arguments/)


```python
import click

@click.command()
@click.option("--count", default=1, help="No. of hello's")
@click.argument("name")
def hello(name="", count=0):
    for _ in range(count):
        print(name)

if __name__ == "__main__":
    hello()
```

**usage**
```bash
python hello_click.py --help   
Usage: hello_click.py [OPTIONS] NAME

Options:
  --count INTEGER  No. of hello's
  --help           Show this message and exit.
```

---

## Group commands
[Commands and Groups](https://click.palletsprojects.com/en/8.1.x/commands/)

```python
import click

@click.group()
def cli():
    pass

@cli.command(help="create dummy")
def create():
    print("create")

@cli.command(help="remove dummy")
def remove():
    print("remove")


if __name__ == "__main__":
    cli()
```

!!! note "Assing command to group"
    Replace `click` decorate with group function name for example `cli`
     

**usage**

```bash
python hello_click.py --help
Usage: hello_click.py [OPTIONS] COMMAND [ARGS]...

Options:
  --help  Show this message and exit.

Commands:
  create  create dummy
  remove  remove dummy
```

---

