[![Build Status](https://travis-ci.org/EmbeddedMontiArc/reporting.svg?branch=master)](https://travis-ci.org/EmbeddedMontiArc/reporting)
[![Build Status](https://circleci.com/gh/EmbeddedMontiArc/reporting.svg?style=shield&circle-token=:circle-token)](https://circleci.com/gh/EmbeddedMontiArc/reporting)

Reporting
========

Creates a report of the quality of all models within a projects-root e.g. `EmbeddedMontiArc` / `MontiSim`.
Each day, Travis automatically calculates the report for `EmbeddedMontiArc` and `MontiSim` and uploads it afterwards to gh-pages.
To add a project (EmbeddedMontiArc/MontiSim), add it to the corresponding String-array in scripts/`gitPullSimple.sh`.

Link to analyzing all component files in the project:
[Component Quality](https://embeddedmontiarc.github.io/reporting/report/cocosReport.html)

Link to all grammars in the project:
[Grammars](https://embeddedmontiarc.github.io/reporting/report/grammarReport.html)

Link to analyzing all Test files in the project:
[Test Quality](https://embeddedmontiarc.github.io/reporting/report/testReport.html)

Code Snippets
======

* zip all model files: `zip -R models '*.m' '*.ema' '*.emam'`
* calculate hash of the zip file: `md5 models.zip`
* rename zip file to `models"hash".zip`
* upload zip to `https://github.com/EmbeddedMontiArc/reporting/tree/gh-pages`

* call `java -jar target/reporting-0.2.1-SNAPSHOT-jar-with-dependencies.jar "ProjectsRoot" -tt true -tc true -zn $models"hash".zip`

* ![Circle CI](https://circleci.com/gh/EmbeddedMontiArc/reporting.png?circle-token=:circle-token)
