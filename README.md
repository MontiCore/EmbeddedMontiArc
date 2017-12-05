[![Build Status](https://travis-ci.org/EmbeddedMontiArc/reporting.svg?branch=master)](https://travis-ci.org/EmbeddedMontiArc/reporting)

Reporting
========

Link to analysing all Test files in the project:
[CheckTestNames](https://embeddedmontiarc.github.io/reporting/report/reportEWT.html?sorts[NameEndsWithTest]=1)

Link to analysing all component files in the project:
[CheckComponents](https://embeddedmontiarc.github.io/reporting/report/report.html?sorts[Valid]=1&sorts[Parse]=-1&sorts[Resolve]=-1)


Code Snippets
======

* zip all model files: `zip -R models '*.m' '*.ema' '*.emam'`
* calculate hash of the zip file: `md5 models.zip`
* rename the zip file to the model, upload it to the reporting and add a coloumn with links to the new zip file and the file to open 
* https://circleci.com/gh/:owner/:repo.png?circle-token=:circle-toke
