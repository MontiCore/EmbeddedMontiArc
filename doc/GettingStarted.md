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

## Contributing
EmbeddedMontiArcStudio is the result of the cooperation between a multitude of university
students and their projects in the context of EmbeddedMontiArc. On that note, a creator of
an EmbeddedMontiArc project is also invited to contribute to this repository with the
integration of her models and/or artifacts. To this end, this project distinguishes between
three different kinds of contribution which are explained in the following.

### Adding Use Cases
EmbeddedMontiArc projects are more often than not working on unique artifacts each being
described by their own particular language. For this reason, a project demonstration is
often accompanied with the creation of a set of demonstration models which can be fed to the
project. The final aim being the integration of an EmbeddedMontiArc project, it would also be
beneficial to have these models available as demonstration project in EmbeddedMontiArcStudio.

In the following, let us assume that a developer wants to integrate a set of models which she
denominated `AutoPilot`. In order to do so, she will have to create an `AutoPilot` folder in
the [`models`](../src/models) directory of this repository. The important aspect here is that
the name of the folder should be **UpperCamelCase**. This newly created folder will act as
model root. Following this, the only thing left for her to do is copying the models into the
`AutoPilot` folder. EmbeddedMontiArcStudio will automatically derive all available
demonstration projects from the directory structure of `models`.

### Adding Scripts
To be written.

<!-- Bound to Button -->

### Adding Dependencies
To be written.