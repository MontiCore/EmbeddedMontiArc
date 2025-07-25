<!-- (c) https://github.com/MontiCore/monticore -->
# Documentation of CI setups

# Travis and Coveralls
1. Enable project builds in travis. [See](https://docs.travis-ci.com/user/getting-started/) for a basic explanation of travis.
2. If no other special settings are required, the travis.yml of a similar project can be used [example](https://github.com/EmbeddedMontiArc/EMAM2Cpp/blob/master/.travis.yml)
   (This will also include Coveralls usage)
3. To deploy to our "github-nexus" a special user is needed and the corresponding credentials need to be added 
   as environment variables to travis to prevent making them publically available.
   
   People who know the credentials:
   
   https://github.com/EvgenyKusmenko
   
   https://github.com/vonwenckstern
   
   https://github.com/sschneiders
   
# CircleCI
1. For CircleCI there must exist a ".cirlceci" folder with a "config.yml" inside.
   The "config.yml" from [example](https://github.com/EmbeddedMontiArc/EMAM2Cpp/blob/master/.circleci/config.yml) 
2. [See](https://circleci.com/docs/enterprise/quick-start/) for an explanation of how to setup a project on the CircleCI [site](https://circleci.com/)

Note that for some reason CircleCI does not work with some projects while Travis does work.

# Badges
[See](https://raw.githubusercontent.com/EmbeddedMontiArc/EMAM2Cpp/master/README.md) for the sample source of a "Readme.md" that can be added to a Github repository. How to obtain the corresponding links for a project for travis is explained [here](https://docs.travis-ci.com/user/status-images/).

For CircleCI see [here](https://circleci.com/docs/1.0/status-badges/) for an explanation.

For Coveralls the badge can be added by replacing `organization` and `projectname` in the following url:
```
[![Coverage Status](https://coveralls.io/repos/github/organization/projectname/badge.svg?branch=master)](https://coveralls.io/github/organization/projectname?branch=master)

```

Note that it may take some time for some badges to update.
Using Ctrl + F5(when using chrome atleast) should show the current badge.
