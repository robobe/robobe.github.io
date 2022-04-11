---
title: Part7 - Operators
description: Variables
date: "2022-04-11"
banner: ../images/yocto.png
tags:
    - yocto
    - 101
---

# Variables
Variables can assign in conf and recipe files

```
VARIABLE = "value"
```


```bash title="variable in conf"
# This command display variable value after config file are parsed
bitbake -e | grep ^<VARIABLENAME>=

# Example
bitbake -e | grep ^BBPATH=
# Result
BBPATH="/home/user/yocto/poky/meta-poky:/home/user/yocto/poky/build:/home/user/yocto/poky/meta:/home/user/yocto/poky/meta-yocto-bsp:/home/user/yocto/meta-external/meta-openembedded/meta-oe:/home/user/yocto/meta-external/meta-openembedded/meta-python"

bitbake -e | grep ^TOPDIR=
TOPDIR="/home/user/yocto/poky/build"
```

```bash title="variable from recipe"
bitbake -e <recipe></recipe> | grep ^<VARIABLE>=

# Example
bitbake -e dropbear | grep ^DEPENDS=
DEPENDS="autoconf-native automake-native libtool-native libtool-cross  virtual/x86_64-poky-linux-gcc virtual/x86_64-poky-linux-compilerlibs virtual/libc zlib virtual/crypt  update-rc.d initscripts virtual/update-alternatives"
```

## Variable soft assignment (?=)
if variable assign as hard assignment the value is lost
if no hard assignment this value are the default one

```
MACHINE ?= "qemuarm"
```

!!! Note
    if ww have multiple soft assignment the **first** one is hold/assign

## Weaker default value (??=)
Assignment is made at the end of the parsing process, rather then immediately

if variable assign as hard assignment the weaker value is lost
if variable assign as default assignment the weaker value is lost

!!! Note
    if ww have multiple weak assignment the **last** one is hold/assign

---

## Variable Expansion
Variable can reference the content of other variable using the `${variable_name}` syntax

```
A = "${B} hello"
B = "${C} world"
C = "linux"
```

!!! Waring
    if variable not declare no expansion will happen
    the string is keep as is