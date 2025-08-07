<!-- (c) https://github.com/MontiCore/monticore -->
# Environment Contribution Language

## Description
This module consists of the files of the Environment Contribution Language (ECL) which is used to describe (parts of)
the IDE's execution environment. The ECL is similar to the Dockerfile language but with slightly altered syntax and it
distinguishes between two different types of models, namely **Component Dockerfile** and **Dockerfile** models. The
former can only use Docker instructions which do not depend on the build context whereas the latter have access to the
full range of the instruction spectrum. 
