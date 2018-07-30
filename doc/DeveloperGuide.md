# Developer Guide
![](https://img.shields.io/badge/Status-Work_In_Progress-blue.svg?longCache=true&style=flat-square)

## Table of Contents
1. [**Introduction**](#1-introduction)
2. [**Architecture**](#2-architecture)
3. [**IDE**](#3-ide)
4. [**Coordinator**](#4-coordinator)
5. [**Batch Scripts**](#5-batch-scripts)
6. [**Executables**](#6-executables)

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

<img src="media/images/ide-coordinator.png" width="500"/>

Figure 2.2 Interaction between IDE and Coordinator.

<img src="media/images/coordinator-batch-scripts.png" width="500"/>

Figure 2.3 Interaction between Coordinator and Batch Scripts.

<img src="media/images/batch-scripts-executables.png" width="500"/>

Figure 2.4 Interaction between Batch Scripts and Executables.

## 3. IDE
<img src="media/images/ide.png" width="500"/>

Figure 3.1 Overview of the plugin functionality of the IDE.

<img src="media/images/plugin.png" width="500"/>

Figure 3.2 Overview of the structure of a plugin.

## 4. Coordinator
<img src="media/images/coordinator.png" width="500"/>

Figure 4.1 Overview of the structure of the Coordinator.

<img src="media/images/constants.png" width="500"/>

Figure 4.2 Overview of the included information.

<img src="media/images/app.png" width="360"/>

Figure 4.3 Overview of the included settings.

## 5. Batch Scripts

## 6. Executables