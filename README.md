[![Packing Windows 64-bit portable application](https://ci.appveyor.com/api/projects/status/e6j0od439jh25ax9?svg=true)](https://ci.appveyor.com/project/vonwenckstern/emastudiobuilder)

# EMAStudioBuilder

This repo contains the scripts and the sources which are used to build EmbeddedMontiArcStudio automatically.

* The `master` branch contains the sources to build EmbeddedMontiArcStudio.
* The `scripts` branch contains the scripts to build based on the `master` branch the EmbeddedMontiArcStudio in appveyor.

In the releases this repo contains the parts which are used by the scripts to build EmbeddedMontiArcStudio.

The building tool chain is the following:
1)	Commit in EMAStudioBuilder master branch triggers Windows appveyor CI service
2)	Appveyor downloads all files specified in the dependencies.txt files
3)	Appveyor downloads the master branch of EMAStudioBuilder again (if you want to overwrite setting files specified in the library zip files)
4)	Appveyor deletes all .gitignore and .gitkeep files as well as the dependencies.txt files
5)	Appveyor downloads WinRaR
6)	Appveyor packs the entire contents to a self-extracting archive
7)	Appveyor creates a new pre-release at [EmbeddedMontiArcStudio](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio) via the Github Release API whereby the text for the release is the same text which is specified in ReleaseNotes.md

In order to not spam the user with too many automatically created pre-releases, we will only keep the latest 5 pre-releases -- Appveyor will delete older automatically created pre-releases (all pre-releases automatically created have a tag like `aut-2018-04-26--19-06`, this way older releases can be automatically removed).
