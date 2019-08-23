<!-- (c) https://github.com/MontiCore/monticore -->
[![Build Status](https://travis-ci.org/EmbeddedMontiArc/reporting.svg?branch=master)](https://travis-ci.org/EmbeddedMontiArc/reporting)
[![Build Status](https://circleci.com/gh/EmbeddedMontiArc/reporting.svg?style=shield&circle-token=:circle-token)](https://circleci.com/gh/EmbeddedMontiArc/reporting)
 [![PPTX-Docu](https://img.shields.io/badge/PPTX--Docu-2018--05--22-brightgreen.svg)](https://github.com/EmbeddedMontiArc/Documentation/blob/master/reposlides/18.05.22.Docu.Reporting.pdf)

Reporting
========

Reporting
========

Creates a report of the quality of all models within a projects-root e.g. `EmbeddedMontiArc`
Each day, Travis automatically calculates the report for `EmbeddedMontiArc` and uploads it afterwards to gl-pages.

[Component Quality](https://monticore.pages.rwth-aachen.de/EmbeddedMontiArc/utilities/reporting/report/componentQuality.html) : 
Report for the CoCo tests of all components.

## How to add a new report

- To add a new Repository Root e.g. `EmbeddedMontiArc`, add the following lines to the gitlab-ci.yml
    - `script`:
        - ./scripts/syncProjects.sh "EmbeddedMontiArc"
        - java -jar target/reporting-0.9-jar-with-dependencies.jar "EmbeddedMontiArc" "-testCoCos" "-timeout" "10"
    - the `options` are:
        - `-testCoCos` in order to add the Report for the Coco tests of all components.
            - `-timeout "t"` in order to set a time limit for building the symbol table 
        - `-testTests` in order to add the Report for all jUnit tests
        - `-grammar` in order to add the Report for all grammars
        - `-m` **essential** : in order to add and not replace the report

- To create a different Report:
    - create a new html file similar to `report/exampleHTML.html` and add it to `report/`
    - create a new js file similar to `report/js/exampleJS.js` and refer to it in your html file and add it to `report/js/`
    - generate a new json file similar to `report/data/exampleJSON.json` and refer to it in your html file and add it to `report/data/`
    - add your html file in the `deploy.sh` script
    - in order to generate the new json file, there are a few tools inside the project's code:
        - `SearchFiles`, `ASTHelper`, `GitHubHelper`
    - in order to customize your report, there are a few tools inside the javascript code
        - `grouping`, `floatingHeader` with group Information, `LogMechanics` and a `childControl`
    - for more information take a look at `cocosReport.html` and `cocosReport.js`
        
        
