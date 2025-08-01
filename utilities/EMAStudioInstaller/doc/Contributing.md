# Contributing
EmbeddedMontiArcStudio is the result of the cooperation between a multitude of university
students and their projects in the context of EmbeddedMontiArc. On that note, a creator of
an EmbeddedMontiArc project is also invited to contribute to this repository with the
integration of her models and/or artifacts. To this end, this project distinguishes between
four different kinds of contribution which are explained in the following.

## Adding Use Cases
EmbeddedMontiArc projects are more often than not working on unique artifacts each being
described by their own particular language. For this reason, a project demonstration is
often accompanied with the creation of a set of demonstration models which can be fed to the
project. The final aim being the integration of an EmbeddedMontiArc project, it would also be
beneficial to have these models available as demonstration project in EmbeddedMontiArcStudio.

In the following, let us assume that a developer wants to integrate a set of models which she
denominated `AutoPilot`. In order to do so, she will have to create an `AutoPilot` folder in
the `models` directory of the target platform (e.g [`models`](../target/src/windows/models) for
Windows). The important aspect here is that the name of the folder should follow the
**UpperCamelCase** convention. This newly created folder will act as model root. Therefore, the
only thing left for her to do is copying the models into the `AutoPilot` folder.
EmbeddedMontiArcStudio will automatically derive all available demonstration projects from the
directory structure of the respective `models` folder.

A computer-assisted integration of new use cases is provided by the
[**Manager**](GettingStarted.md#managing).

## Adding Artifacts
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

Running `yarn install` again will result in the artifact being downloaded and extracted.

In the following, let us assume that a developer has a bundled application named
`visualization-emam`. Following the above guide, the next step would be the creation of a
ZIP archive with the following structure:

```
/
|-- visualization-emam/
    |-- visualization-emam.jar
```

The application being a Java application in nature, the archive has to be uploaded to a
sub-folder `YY.MM.DD.visualization-emam` inside of the `shared` folder, where `YY.MM.DD`
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

Note that `platforms` could further be extended to include unix-based platforms.

A computer-assisted integration of new artifacts is provided by the
[**Manager**](GettingStarted.md#managing).

## Adding Control Scripts
A next aspect to cover is the large variety of execution parameters of EmbeddedMontiArc
projects. Some projects, for instance, need the result of some kind of intermediate
transformation of the models and do not operate on the models themselves. Some others
assume that a different project has been executed beforehand. In order to cover all
these cases, a developer has to write one or more control scripts which coordinate the
execution of the project.

The format of these scripts is Batch (.bat) scripts for Windows as target platform and
Shell (.sh) scripts for unix-based platforms. The advantage of using these types of
scripts is that a developer can make use of the entire functions spectrum of the operating
system. For instance, if a file needs to be copied, a developer can just use `copy` or `cp`
to perform this task. Usually, a script begins with a call to
[`variables`](../target/src/windows/scripts/common/variables.bat), a script which instantiates
useful variables which can be used to ease development of the actual script.

The script will be executed when a certain menu point in EmbeddedMontiArcStudio is selected.
EmbeddedMontiArcStudio derives the correct script from its location and its name. The
following table gives an overview of this convention.

| Menu  | Location | Name{.bat,.sh}  |
| :--- | :--- | :--- |
| Execute Model | scripts/{project}/executing/ | execute |
| Generate WebAssembly | scripts/{project}/emam2wasm/ | emam2wasm |
| Visualize Workspace | scripts/{project}/visualization/ | visualize |
| Report | scripts/{project}/reporting/ | report |
| Report With Streams | scripts/{project}/reporting/ | report.streams |
| Test Workspace | scripts/{project}/testing/ | test.all |
| Test Active Stream | scripts/{project}/testing/ | test |

where `{project}` denotes the name of the demonstration project from the first section.

Apart from the project, the scripts should also be sub-divided by platform as can be seen
[here](../target/src/windows).

A computer-assisted integration of new control scripts and of new entries to above
conventions is provided by the [**Manager**](GettingStarted.md#managing).

## Adding Extensions
Sometimes, the results of an EmbeddedMontiArc project need to be treated further by
EmbeddedMontiArcStudio itself. Some manifestations of this would be the display of test
results, or the opening of a tab in which the results are displayed. Furthermore, some
projects also need functionality which goes beyond the one provided in the previous section.
Some manifestations of this would be the registration of a hook which runs a terminate
script once the studio is closed, or the introduction of new menu points. In order to
complement the studio with these types of functionality, a contributor can develop an
extension.

To successfully develop an extension, a contributor needs to become familiar with
[**TypeScript**](https://www.typescriptlang.org/), [**Inversify**](https://github.com/inversify/InversifyJS),
and especially [**Theia**](https://github.com/theia-ide/theia) as writing an extension for
EmbeddedMontiArcStudio is the same as writing an extension for Theia.

**Note that extensions must be added to the [`extensions`](../extensions) folder and even
though it might be compelling, under no circumstances should the contents of the `ide` and
`packages` folders be modified.**