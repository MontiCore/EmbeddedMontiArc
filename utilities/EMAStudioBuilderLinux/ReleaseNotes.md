<!-- (c) https://github.com/MontiCore/monticore -->
<!--<h1>This version is instable yet, see #14</h1> -->

Requirements:
====
* Windows 64 bit

Usage:
====
* download the EXE file
* and unpack it per double click or with WinRar (it is a WinRar packed EXE, not an installer)
* run ide.bat
* in the opened browser window choose a project of your liking
* edit the models [currently with restrictions]
* click the play button to execute the model
  * **Autopilot:** another tab with the Simulation will open
  * click S2 Native to start the Simulation
  * **Clustering:** another tab with the Cluster Fiddle will start
* click the eye button to generate the graphical output of the models another tab in the browser with the 2D-Visualization will open
  * the visualization does not support component and port arrays yet (visualization works not for Clusterer example)
* click the statistics symbol to generate a report of different metrics with or without stream testing another tab in the browser with the Report will open
* click the green arrow symbol to either run all the tests of the workspace or a single test.
* click the black tick symbols to either run verification of all views or a single view (for this a view needs to be opened)


Changelog:
====

* Fixes issue [#14](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcStudio/issues/14) 

What's under the Hood:
====

* JDK: jdk
* Armadillo: armadillo-8.200.2
* Cluster Fiddle: cluster-fiddle
* GCC Toolchain: mingw64
* Apache Tomcat Web Server: apache-tomcat-9.0.5
* External Libraries: externallibs
* Various Libraries: lib
* Precompiled Armadillo Headers: precompiled
* Chrome Portable: chrome
* EMAM Models: model (Autopilot + Clustering)
* EMAM2CPP Generator: emam2cpp.jar
* NodeJS node.exe
* IDE ide
* EMAM2EMA Generator: emam2ema.jar
* Visualization Generator: montiarc-svggenerator.jar
* Stream Testing
* Report Generator: reporting.jar
* Batch Scripts that glue all this stuff together: scripts
