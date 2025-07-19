<!-- (c) https://github.com/MontiCore/monticore -->
# EmbeddedMontiArcStudio

* **Latest release of old IDE but with complete Features:** https://rwth-aachen.sciebo.de/s/6g3uWJNQcqgnKRc
* **Windows Installer with new IDE:** https://rwth-aachen.sciebo.de/s/ljgKW9cWvL2qMH9?path=%2F2.0.0
  * Please use `C:\EmbeddedMontiArcStudio\` as installation path, if white space as `Program Files` are in path, the tool does not work correct

[![Overview video of EmbeddedMontiArcStudio](https://user-images.githubusercontent.com/30497492/37372601-8b1f3b0a-2713-11e8-860a-e0bb757b6eaa.png)](https://youtu.be/VTKSWwWp-kg)

The aim of this repository is to host the already packed version of EmbeddedMontiArcStudio, a Development Suite for EmbeddedMontiArc. 

This repo only contains self extracting archive-file releases. Please do not push the content of the zip file or the exe into the repo, as it is not compliant with the github rules.
See here how to create new releases: https://help.github.com/articles/creating-releases/

For stable releases later than version 1.4 please insert the EmbeddedMontiArcLogo as logo for the self extracting exe (https://www.youtube.com/watch?v=tWA0VFcwi4c) and replace `ide.bat` in the root folder by a link which also has the EmbeddedMontiArcLogo. (Here is the link to the logo: https://github.com/EmbeddedMontiArc/website/blob/master/EMALogoTransparentWithoutSE.png

Release History
========
* **[v2.0:](https://rwth-aachen.sciebo.de/s/ljgKW9cWvL2qMH9?path=%2F2.0.0)** Installer with new professional IDE
  * BumperBot View Verification and NFPVerification do not work yet
* **[v1.7.0:](https://rwth-aachen.sciebo.de/s/6g3uWJNQcqgnKRc)** Added IDE support for tags
* **[v1.6.5:](https://rwth-aachen.sciebo.de/s/Wli0nHabWAYTgl3)** Added PacMan example and BumperBot View Verification example
* **[v1.5.2:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.5.1)** Updated SVG-Visualization (using new and better algorithm)
* **[v1.5.1:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.5.1)** Fixes issues [#5](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/issues/5) and [#12](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/issues/12)
* **[v1.5:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.5.0)** Replace Octave Backend with Armadillo Backend to reduce download size and increase compilation time
* [v1.4.2:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.4.2) Patches for EmbeddedMontiArcStudio v1.4
* **[v1.4:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.4.0)** Two Examples: Autopilot Model and Image Clusterer Example
* **[v1.3:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.3.0)** IDE with Visualisation, 3D-Simulator and Reporting
* **[v1.2.5:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.2.5)** Merging the previous releases
* **[v1.2:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.2)** Visualisation inside Reporting
* **[v1.1:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.1)** 2D Component Visualisation
* [v1.1:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.1) Reporting (Overview of Model Quality)
* **[v1.05:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.05)** pre-release with IDE (v1.1 - v1.2 contain no IDE)
* **[v1.0:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.0)** EMAM-Compiler
* [v1.0:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.0) Car-Simulator with Physics Engine
* [v1.0:](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/releases/tag/v1.0) 3D-Car Visualisation



Credits
========
* Bernhard Rumpe (Organization-Leader and Sponsor of EmbeddedMontiArc)
* Michael von Wenckstern (Project-Leader: EmbeddedMontiArc-Language, Compiler, Visualisation, IDE, PacMan, SuperMario, and OCL)
* Evgeny Kusmenko (Project-Leader: Simulator, Physic Engine, and 3D-Car)
* Jean-Marc Ronck (IDE)
* Malte Heithoff (Reporting, PacMan)
* Alexander Ryndin (Simulator and Integration of Compiler & Packing first version)
* Sascha Schneiders (EmbeddedMontiArc to C++ Compiler)
* Stefan Brunecker (EmbeddedMontiArcMath to WebAssembly Compiler)
* Armin Mokhtarian (Autopilot Model)
* Manuel Schrick (SVG generation)
* Haller, Philipp (SuperMario)
