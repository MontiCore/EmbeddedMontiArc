# Getting Started

## Table of Contents
* [**Prerequisites**](#prerequisites)
* [**Installation**](#installation)
* [**Compiling**](#compiling)
* [**Building**](#building)
* [**Packaging**](#packaging)
* [**Documentation**](#documentation)
* [**Local Execution**](#local-execution)

## Prerequisites
| ![https://yarnpkg.com](images/yarn.png) | ![https://nodejs.org](images/nodejs.png) |
| :----: | :-----: |
| 1.5.1+ | 8.12.0 |

Furthermore, one needs a C++ compiler and a Python 2.7 distribution for their
respective operating system, such as
[**Visual C++ Build Tools**](https://www.visualstudio.com/downloads/#build-tools-for-visual-studio-2017) and
[**Python 2.7**](https://www.python.org/downloads) for Windows.

## Installation
**Requires**: [**Prerequisites**](#prerequisites)

After having downloaded and installed the prerequisites, a developer can clone this repository
and continue with the installation of this project. The installation is performed by executing
a console command from within the root directory or more precisely the directory where the
`yarn.lock` is located. The console command is the following:

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
**Requires**: [**Building**](#building)

Together with the dependencies loaded during the installation phase of this guide, building
produces all the files necessary to have a runnable system. For each target it, however, holds
true that either some files can be ignored or that some other files must also be included
during deployment in order to have a runnable system in a different environment. An
unexperienced developer might, however, forget some files or include files which are not
needed. For this reason, a command has been introduced which eases the burden on the developer
by automatically packaging the system into a ZIP archive where exactly the necessary files have
been included. In order to execute the packaging process, a developer has to enter the
following command in the root directory:

```bash
yarn run package
```

The resulting archive will be generated into the `dist` folder in the `target` directory.

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
**Requires**: [**Building**](#building)

Transferring the output files to the server responsible for their distribution could become
quite inefficient when done after each implementation phase in order to test the usability of
new features. For this reason, this project is shipped with its own server implementation
based on [**Express**](http://expressjs.com) in order to locally test the functionality.
Starting this server is performed with the following console command executed in the root
directory:

```bash
yarn run start
```

The port used for the server defaults to `3005` and the implementation can be visited on
[http://localhost:3005](http://localhost:3005) with any of the
[supported browsers](../README.md#browser-support).ngStarted.md#local-execution)