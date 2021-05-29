<!-- (c) https://github.com/MontiCore/monticore -->
# Language Client Plugin

## Description
This module consists of the files related to the Maven plugin operating on the LCL.

## Plugin Configuration Options
| Option Name | Description | Required |
| --- | --- | :---: |
| rootModel | A qualified name to a LCL model. | Yes |
| grammar | The qualified name of the grammar on which the language client should work upon. | Yes |
| outputDirectory | The directory to which the files should be generated to. | No |
| handCodedPaths | A list of paths acting as roots for the handwritten code detection. | No |

## Sol Package Directories Options
| Option Name | Description |
| --- | --- |
| models | A relative path pointing to a directory acting as model path. |
| templates | A relative path pointing to a directory in which templates are stored. |
| server | A relative path pointing to a directory in which the copied language server should be stored. |
