<!-- (c) https://github.com/MontiCore/monticore -->
# Language Server Plugin

## Description
This module consists of the files related to the generation of a simple language server for a given language given form
by an executable JAR.

## Plugin Configuration Options
| Option Name | Description | Required |
| --- | --- | :---: |
| handCodedPaths | A list of paths acting as roots for the handwritten code detection. | No |
| grammar | The qualified name of the grammar on which the language server should operate on. | Yes |
| outputDirectory | The directory to which the files should be generated to. | No |
