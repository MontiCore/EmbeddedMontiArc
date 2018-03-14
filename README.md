[![Build Status](https://travis-ci.org/EmbeddedMontiArc/EMAM2RosCpp.svg?branch=master)](https://travis-ci.org/EmbeddedMontiArc/EMAM2RosCpp)
[![Build Status](https://circleci.com/gh/EmbeddedMontiArc/EMAM2RosCpp/tree/master.svg?style=shield&circle-token=:circle-token)](https://circleci.com/gh/EmbeddedMontiArc/EMAM2RosCpp/tree/master)
[![Maintainability](https://api.codeclimate.com/v1/badges/d997995a55ef427d9467/maintainability)](https://codeclimate.com/github/EmbeddedMontiArc/EMAM2RosCpp/maintainability)
[![Coverage Status](https://api.codeclimate.com/v1/badges/d997995a55ef427d9467/test_coverage)](https://codeclimate.com/github/EmbeddedMontiArc/EMAM2RosCpp/test_coverage)

# EMAM2RosCpp

## Warning
This generator is part of an composite generator and does not create an executable. Look at https://github.com/EmbeddedMontiArc/EMAM2Middleware if you want to generate one.

It also needs versions of other EMAM libraries that are not yet distributed via Maven. Workaround: Download these branches and build each via 'mvn clean install -U -s settings.xml'
* EmbeddedMontiArc/RosPort: https://github.com/EmbeddedMontiArc/EmbeddedMontiArc/tree/RosPort
* EmbeddedMontiArcMath/RosPort: https://github.com/EmbeddedMontiArc/EmbeddedMontiArcMath/tree/RosPort
* EMAM2RosMsg: https://github.com/EmbeddedMontiArc/EMAM2RosMsg

or use this script:
```bash
#!/bin/bash
#fail if any of the commands fail
set -e

git clone https://github.com/EmbeddedMontiArc/EmbeddedMontiArc
git clone https://github.com/EmbeddedMontiArc/EmbeddedMontiArcMath
git clone https://github.com/EmbeddedMontiArc/EMAM2RosMsg

cd EmbeddedMontiArc
git checkout RosPort
mvn clean install -U -s settings.xml
cd ..

cd EmbeddedMontiArcMath
git checkout RosPort
mvn clean install -U -s settings.xml
cd ..

cd EMAM2RosMsg
mvn clean install -U -s settings.xml
cd ..
```
