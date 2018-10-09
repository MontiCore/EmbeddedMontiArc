# Getting Started

## Prerequisites
| ![https://yarnpkg.com](images/yarn.png) | ![https://nodejs.org](images/nodejs.png) |
| :----: | :-----: |
| 1.5.1+ | 8.12.0 |

Furthermore, one needs a C++ compiler and a Python 2.7 distribution for their
respective operating system, such as
[**Visual C++ Build Tools**](https://www.visualstudio.com/downloads/#build-tools-for-visual-studio-2017) and
[**Python 2.7**](https://www.python.org/downloads) for Windows.

## Installation
After having downloaded and installed the prerequisites, a developer can clone this repository
and continue with the installation of this project. The installation is performed by executing
a console command from within the root directory or more precisely the directory where the
`yarn.lock` is located. The console command is the following:

```bash
yarn install
```

Depending on the system, the installation might take a few minutes.

## Packaging
Packaging in the context of this repository denotes the generation of an installer for
EmbeddedMontiArcStudio for a specific target platform. In order to successfully execute
the packaging process, all artifacts should have been downloaded and extracted during the
installation process. Two different packaging modes can be distinguished, namely `package`
and `package:dry`. The former triggers the generation of all necessary artifacts including
the installer itself whereas the latter excludes the installer. The `package:dry` mode
can be used to test EmbeddedMontiArcStudio before packaging it into a fully-fledged
installer. To execute either of the modes, a developer has to execute the following console
commands in the root directory of this repository:

```bash
yarn run package[:dry]
```