[![Build Status](https://travis-ci.org/EmbeddedMontiArc/reporting.svg?branch=master)](https://travis-ci.org/EmbeddedMontiArc/reporting)
[![Build Status](https://circleci.com/gh/EmbeddedMontiArc/reporting.svg?style=shield&circle-token=:circle-token)](https://circleci.com/gh/EmbeddedMontiArc/reporting)

Reporting
========

Creates a report of the quality of all models within a projects-root e.g. `EmbeddedMontiArc` / `MontiSim` / `MontiCore`.
Each day, Travis automatically calculates the report for `EmbeddedMontiArc` and `MontiSim` and uploads it afterwards to gh-pages.

[Component Quality](https://embeddedmontiarc.github.io/reporting/report/cocosReport.html) : 
Report for the CoCo tests of all components.

[Grammars](https://embeddedmontiarc.github.io/reporting/report/grammarReport.html) : 
Report of all grammars.

[Test Quality](https://embeddedmontiarc.github.io/reporting/report/testReport.html) : 
Report of all jUnit tests.

## How to add a new report

- To add a new Repository Root e.g. `EmbeddedMontiArc`, add the following lines to the travis.yml
    - `script`:
        - -bash ./scripts/gitpull.sh `New_Repository_Root`
    - `after success`:
        - -`new_Zip_Var`=$(createZip `New_Repository_Root`)
        - OR
        - -`new_Zip_Var`=$(createGrammarZip `New_Repository_Root`)
    - `after success`:
        - -java -jar target/reporting-`current_Version`-jar-with-dependencies.jar `New_Repository_Root` `options` -m
    - `after success`:
        - -bash ./scripts/deploy.sh `oldZipVars` `$new_Zip_Var`
    - the `options` are:
        - `-tt` in order to add the Report for all jUnit tests
        - `-tc` `-zn` `$newZipVar` in order to add the Report for the Coco tests of all components.
            - `-svg` to also create the visualisation for all components
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
        
        