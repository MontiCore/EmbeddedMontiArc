# 2D Visualization for EmbeddedMontiArcMath

<p align="center">
    <a>
        <img src="https://img.shields.io/badge/Version-0.1.0-blue.svg?longCache=true&style=flat-square"/>
    </a>
    <a href="https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fshared%2F18.07.20.visualization-emam&files=visualization-emam.zip">
        <img src="https://img.shields.io/badge/Download-18.07.20-blue.svg?longCache=true&style=flat-square"/>
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
To be written.

## Demonstration
![](doc/media/videos/VisualizationEMAM.mp4)

[Try it out](https://embeddedmontiarc.github.io/VisualizationEMAM/)

## Command Line Interface
| Short | Long         | Description                                     |
| :---: | :---:        | :---                                            |
| -m    | --model      | Qualified Name of the Main Component Instance.  |
| -mp   | --modelPath  | Path to the package root of the models.         |
| -out  | --outputPath | Path to the output directory.                   |

## URL Syntax
```
< URL > ::= < Base URL > "#" < Qualified Name of Component Instance > ":" < Visualization Mode > ["&" < Qualified Name of Component harboring the Math Implementation > ":" < Math Mode > [":" < Start Line > [":" < End Line >] ] ]

< Visualization Mode > ::= 0 | 1 | 2 | 3

< Math Mode > ::= 0 | 1
```

## Browser Support
| ![Google Chrome](doc/media/images/chrome.png) | ![Mozilla Firefox](doc/media/images/firefox.png) | ![Safari](doc/media/images/safari.png) | ![Opera](doc/media/images/opera.png) | ![Microsoft Edge](doc/media/images/edge.png) | ![Internet Explorer](doc/media/images/ie.png) |
| :--------------------: | :--------------------: | :--------------------: | :--------------------: | :--------------------: | :---: |
| 57+ :heavy_check_mark: | 52+ :heavy_check_mark: | 11+ :heavy_check_mark: | 44+ :heavy_check_mark: | 16+ :heavy_check_mark: | :x:   |

| ![PC](doc/media/images/pc.png) | ![Smartphones](doc/media/images/smartphone.png) | ![Tablets](doc/media/images/tablet.png) |
| :----------------------: | :---------------------------------------: | :-------------------------------: |
| :heavy_check_mark:       | :heavy_check_mark:                        | :heavy_check_mark:                |


## License
Copyright (C) 2018 SE RWTH.

A concrete license is to be discussed.