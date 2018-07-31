# Developer Guide
![](https://img.shields.io/badge/Status-Work_In_Progress-blue.svg?longCache=true&style=flat-square)

## Table of Contents
1. [**Introduction**](#1-introduction)
2. [**Architecture**](#2-architecture)
3. [**IDE**](#3-ide)
4. [**Coordinator**](#4-coordinator)
5. [**Batch Scripts**](#5-batch-scripts)
6. [**Executables**](#6-executables)
7. [**Models**](#7-models)
8. [**Scenario**](#8-scenario)
9. [**Frequently Asked Questions**](#9-frequently-asked-questions)

## 1. Introduction
EMAStudioBuilder, as the name suggests, is the project responsible for building
EmbeddedMontiArcStudio.

EmbeddedMontiArcStudio is a collaborative effort of the EmbeddedMontiArc subgroup which binds
together most if not all of the projects created by the group's developers. It acts as
Integrated Development Environment (IDE) for MontiCAR, a Domain-Specific Language (DSL) family
for the architectural and behavioral description of cyber-physical systems, such as
autonomously driving vehicles.

Due to the large variety of different projects which have been or will be integrated into
EmbeddedMontiArcStudio and which range from Java to C++ applications with a multitude of
different dependencies, EmbeddedMontiArcStudio has been developed with generality in mind. As
a consequence, however, there has been an increase in complexity with respect to the
integration of new projects and this can be quite overwhelming for new developers. This
developer guide aims at compensating for this fact by depicting the most important actors and
the necessary steps to integrate a new project.

## 2. Architecture
<img src="media/images/architecture.png" width="600"/>

Figure 2.1 Overview of EmbeddedMontiArcStudio's architecture.

## 3. IDE
<img src="media/images/ide.png" width="500"/>

Figure 3.1 Overview of the plugin functionality of the IDE.

<img src="media/images/plugin.png" width="500"/>

Figure 3.2 Overview of the structure of a plugin.

## 4. Coordinator
<img src="media/images/ide-coordinator.png" width="500"/>

Figure 4.1 Interaction between IDE and Coordinator.

<img src="media/images/coordinator.png" width="500"/>

Figure 4.2 Overview of the structure of the Coordinator.

<img src="media/images/constants.png" width="500"/>

Figure 4.3 Overview of the information included in Constants.

<img src="media/images/app.png" width="360"/>

Figure 4.4 Overview of the settings included in App.

## 5. Batch Scripts
<img src="media/images/coordinator-batch-scripts.png" width="500"/>

Figure 5.1 Interaction between Coordinator and Batch Scripts.

## 6. Executables
As established in the previous section, the Coordinator executes Batch Scripts designed to
prepare and execute an integrated project. In order to achieve this, these scripts make use of
installed executables. Available executables are, amongst others, the executables provided by
the Operating System, but also executables installed as dependencies for EmbeddedMontiArcStudio,
such as java or g++. An overview of this can be seen in Figure 6.1.

In order to add an executable as dependency, a developer has to add an entry to
[`dependencies.txt`](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/EMAStudioBuilder/blob/master/dependencies.txt#L31)
which links to a ZIP archive containing the executable and all of its dependencies. Furthermore,
in order to make the executable available as command in batch scripts, one has to add its
parent folder to the
[`PATH`](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/EMAStudioBuilder/blob/master/EmbeddedMontiArcStudio/scripts/shared/variables.bat#L16)
environmental variable. Alternatively, one has to specify the relative or absolute path to the
executable.

<img src="media/images/batch-scripts-executables.png" width="500"/>

Figure 6.1 Interaction between Batch Scripts and Executables.

## 7. Models

## 8. Scenario
Imagine that you are in the following situation. You are a developer for the EmbeddedMontiArc
subgroup and you have developed a language called `EmbeddedMontiArcBLA` which is the `BLA`
language embedded into the EmbeddedMontiArc language. BLA stands for **B**iologically
**L**inked **A**rchitecture (what else did you expect?) and is a language for the
architectural description of biological components. Based on this newly created language
(which if I may say so might shape the future of cyber-biological systems), you have developed
a Java application which you called `VisualizationBLA` and which visualizes the links between
different biological components. Like a good Software Engineer, you have created a collection
of EmbeddedMontiArcBLA models which model the behavior of an intelligent exoskeleton
(awesome stuff) in order to test your application (Wait, you did not test your application?
There is still time, I will not tell on you). Your fully functional and tested application
expects three inputs. The first one being the fully qualified name of the main model, namely
`de.rwth.se.Exoskeleton`, the second one being the model path
`C:\Users\Peter\Exoskeleton\input` and the last one being the output path
`C:\Users\Peter\Exoskeleton\output` (Wait, your name is not Peter? Well, it is now). Your
supervisor, the stakeholders (or for whomever you are developing for) like(s) your project.
You are already expecting a good grade or a promotion. Heck, maybe you are already planning on
going on vacation after all that hard work you have done. But hold your horses, life is not
that simple, because you have been given a last task to perform: integrating your project into
EmbeddedMontiArcStudio. Now, that is a scary task if I have ever seen one, but do not worry,
for I will guide you through the necessary steps.

## 9. Frequently Asked Questions