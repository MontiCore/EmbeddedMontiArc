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
To be written.

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
the name of the folder should follow the **UpperCamelCase** convention. This newly created
folder will act as model root. Therefore, the only thing left for her to do is copying
the models into the `AutoPilot` folder. EmbeddedMontiArcStudio will automatically derive all
available demonstration projects from the directory structure of `models`.

### Adding Artifacts
Apart from models, an EmbeddedMontiArc project also often consists of an executable part or
application which performs some tasks on a given input. Adding this executable part is a
four-step process.

First, the application has to be bundled into an executable format. In the case of a Java
application, for instance, this format would be
[a JAR where all dependencies have been included](https://stackoverflow.com/questions/574594/how-can-i-create-an-executable-jar-with-dependencies-using-maven).
In the case of a C++ or similar application, on the other hand, the format would be an
executable with or without DLLs.

Following this, the above-created bundle has to be packed into a **ZIP** archive. The
structure of this ZIP archive should follow the convention that the root of the archive
consists of a folder with the artifacts' name. The executable part should then be located
inside this folder.

This archive then needs to be made publicly available in order to allow other developers to
build upon the newly added functionality. To this end, a cloud storage space on
[Sciebo](https://www.sciebo.de/) has been made available where a developer can upload the
archive. If a developer should not have access to the cloud storage, she should ask her
supervisor whether she can unlock access for her. Inside the cloud storage, there should be
a folder `dependencies` with three sub-folders `windows`, `linux` and `shared`. The archive
has to be uploaded to either of the folders depending on whether the project will run on any
operating system or only on a specific one. The target of the upload should be a folder
`YY.MM.DD.artifact-name` inside one of three previously mentioned folders where `YY.MM.DD`
denotes the day of the upload and `artifact-name` the name of the artifact.

The last step is telling EMAStudioInstaller that a new artifact has been made available. To
this end, the configuration file
[`dependencies.json`](../configs/dependencies.json) has to be extended by a new entry. An
entry in `dependencies.json` has the following structure:

```json
{
  "//": "...",
  "platforms": ["...", "..."],
  "from": "...",
  "to": "..."
}
```

`//` introduces a commentary field which can be used to add any kind of information about
the artifact about to be added. `platforms` is an array of platform identifiers which are
used to tell the system for which platforms the artifact should be downloaded and included
in the overall result. A list of the available platform identifiers can be found
[here](https://nodejs.org/api/process.html#process_process_platform). `from` introduces a
field where a developer has to specify the url from which the system should download the
artifact. `to` specifies the relative location to which the archive should be extracted
after the download phase. In most cases, `to` should be `resources` as this denotes the
root for EmbeddedMontiArcStudio's resources.

In the following, let us assume that a developer has a bundled application named
`visualization-emam`. Following the above guide, the next step would be the creation of a
ZIP archive with the following structure:

```
/
|-- visualization-emam/
    |-- visualization-emam.jar
```

The application being a Java application in nature, the archive has to be uploaded to a
sub-folder `YY.MM.DD.visualization-emam` inside of the `shared` folder where `YY.MM.DD`
denotes the date of the upload. Lastly, `dependencies.json` has to be extended by the
following entry:

```json
{
  "//": "VisualizationEMAM",
  "platforms": ["win32"],
  "from": "https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fshared%2FYY.MM.DD.visualization-emam&files=visualization-emam.zip",
  "to": "resources"
}
```

Note that the platforms could further be extended to include unix based platforms.

### Adding Control Scripts
To be written.

<!-- Bound to Button -->