<!-- (c) https://github.com/MontiCore/monticore -->
# Artifact Contribution Language Plugin

## Description
This module consists of the files related to the Maven plugin operating on the ACL.

## Plugin Configuration Options
| Option Name | Description | Required |
| --- | --- | :---: |
| rootModels | A list of qualified names to the ACL models. | Yes |
| handCodedPaths | A list of paths acting as roots for the handwritten code detection. | No |
| outputDirectory | The directory to which the files should be generated to. | No |

## Sol Package Directories Options
| Option Name | Description |
| --- | --- |
| models | A relative path pointing to a directory acting as model path. |
| artifacts | A relative path pointing to a directory in which copied artifacts should be stored. |
