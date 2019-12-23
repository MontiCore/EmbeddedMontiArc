<!-- (c) https://github.com/MontiCore/monticore -->
# 2D Visualization for EmbeddedMontiArcMath

<p align="center">
    <a>
        <img src="https://img.shields.io/badge/Version-0.1.1-blue.svg?longCache=true&style=flat-square"/>
    </a>
    <a href="https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fshared%2F18.10.02.visualization-emam&files=visualization-emam.zip">
        <img src="https://img.shields.io/badge/Download-18.10.02-blue.svg?longCache=true&style=flat-square"/>
    </a>
</p>

## Table of Contents
* [**Description**](#description)
* [**Demonstration**](#demonstration)
* [**Command Line Interface**](#command-line-interface)
* [**URL Syntax**](#url-syntax)
* [**Requirements**](doc/Requirements.md)
* [**Browser Support**](#browser-support)
* [**License**](#license)

## Description
Whether textual or graphical models are better for software development in general is still an
ongoing debate. Nonetheless, textual models have a multitude of advantages compared to their
graphical peers. Textual models, for instance, can be edited using a simple editor which is
available on every operating system whereas graphical models more often than not need
commercial software. Version Control, including the visualization of the differences between
versions, is easier to implement on textual models than on graphical models due to the sheer
amount of available programs which are specialized on such a task.

In the context of Component and Connector (C&C) models, however, one can realize a disadvantage
of textual models. With the increasing size of the models, it often becomes difficult to keep
the overview on how components are interacting with each other. For a developer using graphical
models, on the other hand, this task is achieved with ease because she can see the components
and their connections visually in front of her.

In order to compensate for this fact, this project and [one of its
dependencies](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/visualisation)
generate a visual representation of textual EmbeddedMontiArcMath models showing the
relationships between components.

## Demonstration
![](doc/media/videos/VisualizationEMAM.mp4)

[Try it out](https://embeddedmontiarc.github.io/webspace/Models2018.EXE/htmlModels/)

## Command Line Interface
| Short | Long         | Description                                     |
| :---: | :---:        | :---                                            |
| -m    | --model      | Qualified Name of the Main Component Instance.  |
| -mp   | --modelPath  | Path to the package root of the models.         |
| -out  | --outputPath | Path to the output directory.                   |

## URL Syntax
```
<URL> ::= <Base URL> "#" <Qualified Name of Component Instance> ":" <Visualization Mode> ["&" <Qualified Name of Component harboring the Math Implementation> ":" <Math Mode> [":" <Start Line> [":" <End Line>] ] ]

<Visualization Mode> ::= 0 | 1 | 2 | 3

<Math Mode> ::= 0 | 1
```

* After opening an atomic component, you can just add `:2` at the end of the URL to highlight line number `2` for issues.
* If you add `:2:4` at the end of the URL, then you highlight the line numbers `2`, `3`, and `4`.

## Browser Support
| ![Google Chrome](doc/media/images/chrome.png) | ![Mozilla Firefox](doc/media/images/firefox.png) | ![Safari](doc/media/images/safari.png) | ![Opera](doc/media/images/opera.png) | ![Microsoft Edge](doc/media/images/edge.png) | ![Internet Explorer](doc/media/images/ie.png) |
| :--------------------: | :--------------------: | :--------------------: | :--------------------: | :--------------------: | :---: |
| 57+ :heavy_check_mark: | 52+ :heavy_check_mark: | 11+ :heavy_check_mark: | 44+ :heavy_check_mark: | 16+ :heavy_check_mark: | :x:   |

| ![PC](doc/media/images/pc.png) | ![Smartphones](doc/media/images/smartphone.png) | ![Tablets](doc/media/images/tablet.png) |
| :----------------------: | :---------------------------------------: | :-------------------------------: |
| :heavy_check_mark:       | :heavy_check_mark:                        | :heavy_check_mark:                |


## License

[(c) https://github.com/MontiCore/monticore](https://github.com/MontiCore/monticore)


A concrete license is to be discussed.
