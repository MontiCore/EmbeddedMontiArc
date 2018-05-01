# <p align="center">Getting Started<p>

## Prerequisites
| ![https://yarnpkg.com](images/yarn.svg) | ![https://nodejs.org](images/nodejs.svg) |
| :----: | :-----: |
| 1.5.1+ | 0.12.0+ |

Furthermore, one might need a C++ compiler and a Python 2.7 distribution for their
respective operating system, such as
[**Visual C++ Build Tools**](http://landinghub.visualstudio.com/visual-cpp-build-tools) and
[**Python 2.7**](https://www.python.org/downloads) for Windows.

## Installation
After having downloaded and installed the prerequisites, a developer can clone this repository
and continue with the installation of this project. The installation is performed by executing
two console commands from within the root directory or more precisely the directory where the
`yarn.lock` is located. These two commands written as one are:

```bash
yarn install && yarn run download
```

Depending on the system, the installation might take a few minutes.

## Building
The main programming language used for the implementation of this project as well as of most of
its dependencies is [**TypeScript**](https://www.typescriptlang.org). TypeScript is a
programming language developed by Microsoft and is a superset of the JavaScript programming
language which introduces new syntactic constructs as well as static typing to its subset,
which makes code more maintainable. Nonetheless, TypeScript is not directly executed in its
respective runtime environment but is translated to JavaScript using the compiler that comes
shipped with it. In order to execute this compiler on the files of this project, a developer
has to run the following console command in the root directory:

```bash
yarn run compile
```

The execution of this command, however, results in a large amount of JavaScript files which
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

This console command implicitly calls the `compile` command and the output is generated in
the `docs` folder of the root directory.

## Documentation
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
yarn run documentation
```

The output path of the documentation defaults to the `docs` folder inside the root
directory.

## Publishing
Publishing in the context of this project denotes the generation of all the files necessary
for the execution of and understanding for the development on this project. Therefore, it
denotes the combination of all the commands described in the [__Building__](#building) and
[__Documentation__](#documentation) sections of this description. A shorthand command which
executes all these commands is the following:

```bash
yarn run preupload
```

The output path of all the generated files defaults to the `docs` folder inside the root
directory of this project.

## Local Execution
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
[supported browsers](../README.md#browser-support).