<!-- (c) https://github.com/MontiCore/monticore -->
# ArtifactAnalysis

The website needs two different inputs. The implemented and extracted results and the specification against the implementation is run.

The implemented are usually a Merged.OD and should be set with its .od and .json path in the analysedOD.json file in the assets Folder. It is also needed to put another .json file of the Merged.OD inside this folder which contains a forest representation of it. This can be created with the artifact-language-tool@^1.2.0-SNAPSHOT.

It is possible to input multiple specifications with the moduleAnalyses.json file, which takes an input array (separated with comma) where the ModuleAnalysis.json files reside. 
