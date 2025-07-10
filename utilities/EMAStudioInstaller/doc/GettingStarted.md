# Getting Started

## Table of Contents
* [**Prerequisites**](#prerequisites)
* [**Installation**](#installation)
* [**Compiling**](#compiling)
* [**Building**](#building)
* [**Packaging**](#packaging)
* [**Documentation**](#documentation)
* [**Local Execution**](#local-execution)
* [**Managing**](#managing)

## Prerequisites
| ![https://yarnpkg.com](images/yarn.png) | ![https://nodejs.org](images/nodejs.png) |
| :----: | :-----: |
| [**1.5.1+**](https://yarnpkg.com) | [**10.14.2**](https://nodejs.org) |

Furthermore, one needs a C++ compiler and a Python 2.7 distribution for their
respective operating system, such as
[**Visual C++ Build Tools**](https://www.visualstudio.com/downloads/#build-tools-for-visual-studio-2017) and
[**Python 2.7**](https://www.python.org/downloads) for Windows.

## Installation
**Requires**: [**Prerequisites**](#prerequisites)

After having downloaded and installed the prerequisites, a developer can clone this repository
and continue with the installation of this project. The installation is performed in two steps.

First, the submodules of the repository need to be fetched. In order to perform this step, a
developer has to execute the following console command from within the root directory or more
precisely the directory where `yarn.lock` is located:

```bash
git submodule update --init
```

Secondly, the dependencies of the project need to be downloaded and processed. In order to
perform this step, a developer has to execute the following console command from within the
root directory:

```bash
yarn install
```

Depending on the system, the installation might take a few minutes.

## Compiling
**Requires**: [**Installation**](#installation)

The main programming language used for the implementation of this project as well as of most of
its dependencies is [**TypeScript**](https://www.typescriptlang.org). TypeScript is a
programming language developed by Microsoft and is a superset of the JavaScript programming
language which introduces new syntactic constructs as well as static typing to its subset,
which makes code more maintainable. Nonetheless, TypeScript is not directly executed in its
respective runtime environment but is translated to JavaScript using the compiler that comes
shipped with it.

In order to execute the compiling process, a developer has to run the following console
command in the root directory:

```bash
yarn run compile
```

## Building
**Requires**: [**Compiling**](#compiling)

The execution of the compiler, however, results in a large amount of JavaScript files which
all have to be served to the client in order to guarantee a functional system. However, the
HyperText Transfer Protocol (HTTP) used for the transmission of such files limits the client
to a maximum of six parallel Transmission Control Protocol (TCP) connections resulting in a
parallel transfer of six files. As a consequence, the user would experience quite an
increased page load time.

In order to overcome this limitation, the JavaScript files and the assets used in this
project are combined into bundles using [**WebPack**](https://webpack.js.org/). As a result,
less files have to be transferred to the client and the total page loading time decreases.

In order to execute the bundling process, a developer has to run the following console
command in the root directory:

```bash
yarn run build
```

There is also a second build command which will generate more files which include more
debug information which is useful during the development phase. Similar to the first one,
the command is as follows:

```bash
yarn run build:development
```

## Packaging
**Requires**: [**Building**](#compiling)

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

## Documentation
**Requires**: [**Compiling**](#compiling)

One of the key aspects which makes a piece of software maintainable and extensible is how well
it is documented. The quality of a documentation is both influenced from its content and its
presentation. For instance, a documentation where a developer cannot deduce whether a class
is extended or not is lower in quality as a documentation where this is possible in an instant.

In order to increase the quality of the documentation, the code is complemented with
annotations and the documentation is generated from the knowledge gained from the combination
of both annotations and code analysis.

For TypeScript one of the available generators for documentations is
[**TypeDoc**](http://typedoc.org/) and it is also the one used in this project. A developer
can generate the documentation from the files of the project by running the following console
command from the root directory:

```bash
yarn run docs
```

The output path of the documentation defaults to the `docs` folder inside the respective
package or extension.

## Local Execution
**Requires**: [**Packaging**](#packaging)

In theory, each compile and build step would have to be followed by a packaging step in order
to try out newly implemented functionality. However, the time that a packaging step consumes
increases in duration with each artifact to be included. As a consequence, the packaging step
should only be executed when absolutely necessary. Nonetheless, a developer should be able to
try out new features as quick as possible.

For this reason, a procedure has been put into place which enables the injection of newly
built files into an already existing dry-packaged version of this repository. In order to
trigger the injection and the execution of the application, a developer needs to execute the
following command from the root directory:

```bash
yarn start
```

## Managing
**Requires**: [**Installation**](#installation), [**Contributing**](Contributing.md)

The integration of a project can sometimes be overwhelming for a developer which is not
familiar with the architecture of this repository. As a consequence, integrations are
often prone to errors. Furthermore, manually performing the steps necessary for an
integration is often tedious and time-consuming.

However, creating error-free artifacts within a small time frame is a valuable asset in
Software Engineering. For this reason, projects are often shipped with generators which
produce error-free output on given data within seconds.

On that note, this repository also implements a
[**Management Tool**](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/EMASInstallerManager)
which enables a computer-assisted integration of a project.

In order to launch the manager, a developer has to run the following console command from
the root directory:

```bash
yarn run manage
```