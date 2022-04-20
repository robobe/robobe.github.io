---
title: Part1 - Yocto tutorial
description: Build custom linux image with yocto
date: "2022-04-06"
banner: ../images/yocto.png
tags:
    - yocto
    - 101
---

## Elements of embedded linux
- Toolchain: Compiler and other tools needed to build code for target device
- Bootloader: The program that init the board and load the linux kernel
- Kernel
- Root F.S: Contain all programs and libraries for our system

## Yocto
Yocto project provide tools for create custom linux distributions for any H.W


![](images/yocto_block.drawio.png)


## Poky
Poky is a reference/example linux distribution create by yocto  

The poky repository is an aggregation of several repositories: 
- openembedded-core
- bitbake
- meta-poky
- yocto-docs

Poky = Bitback + Metadata

!!! Note
    - Yocto is a origination like `canonical`
    - Poky is like `ubuntu`


!!! Note
    the 'meta' folder in the poky repository corresponds to the Openembedded Core layer, e.g. https://git.openembedded.org/openembedded-core/. OE Core includes the main components/recipes that are widely used for any configuration.

## Metadata
- Build instructions
- From where obtain sources 
- Changes / additions to sources

Metadata in yocto is collection of
- configuration file (.conf)
- Recipes (.bb and .bbappend)
- Class (.bbclass)
- Includes (.inc)

!!! Note
    he 'meta-oe' folder in meta-openembedded is another layer typically called the 'openembedded' layer. meta-oe contains a large amount of additional recipes.

## Bitback
- Task scheduler


## BSP: Board Support Package
BSP is collection of information that defines how to support particular HW

- HW futures
- Kernel configuration
- Additional S.W

!!! Note
    poky support by default x86, x86-64, BeagleBone devices (has BSP for each H.W type)
---

# Reference
- [udemy yocto]()