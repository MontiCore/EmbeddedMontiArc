<!-- (c) https://github.com/MontiCore/monticore -->
<!--[![Packing Windows 64-bit portable application](https://ci.appveyor.com/api/projects/status/e6j0od439jh25ax9?svg=true)](https://ci.appveyor.com/project/vonwenckstern/emastudiobuilder)-->

# EMAStudioBuilder

**This project is no longer supported and is classified as legacy software. [EMAStudioInstaller](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/EMAStudioInstaller) should be used instead.**

<hr/>
<hr/>

**Important: Never delete any Github Release.** Also if you do not need the release anymore, please let it there otherwise the version control of `dependencies.txt` does not work. If you want to update any binary (e.g. jar), then just upload the new JAR in a new release and then update the link in the `dependencies.txt` file.

The archives you upload to the releases **should not contain .git** files/folders or any other development information (such as `.classpath`, `.settings` or IntelliJ project information, or SVN meta-data).
The self-extracting archive will **never use maven**, please build up-front with maven a JAR with all dependencies and add this to the 
release.

This repo contains the scripts and the sources which are used to build EmbeddedMontiArcStudio automatically.

* Included in a release will be:
  1. EmbeddedMontiArcStudio/*
  2. ide.bat
* If additional files or folders on the project root level should be included in a release, add them [here](https://github.com/EmbeddedMontiArc/EMAStudioBuilder/blob/master/build/config.ps1).
* If additional dependencies should be included, add them to [dependencies.txt](https://github.com/EmbeddedMontiArc/EMAStudioBuilder/blob/master/dependencies.txt).

The building tool chain is the following:
1.	Commit in EMAStudioBuilder master branch triggers Windows appveyor CI service
2.  Appveyor calls the building process
    1.  Builder will download all files specified in the dependencies.txt
    2.  Extract them according to the relative paths specified in dependencies.txt
    3.  Add (and potentially override with) specified project files from the master branch
    4.  Pack all to a self-extracting archive
3.	Appveyor then creates a new pre-release at [EmbeddedMontiArcStudio](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio) via the Github Release API whereby the text for the release is the same text which is specified in ReleaseNotes.md

In order to not spam the user with too many automatically created pre-releases, we will only keep the latest 5 pre-releases -- Appveyor will delete older automatically created pre-releases (all pre-releases automatically created have a tag like `aut-2018-04-26--19-06`, this way older releases can be automatically removed).


One can also create a release locally by executing build/make.bat.
For this .NET 3.0 / PowerShell 2.0 (Windows7) or higher is required.
