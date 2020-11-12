/* (c) https://github.com/MontiCore/monticore */
package lifelongLanguageLearning;
conforms to dltag.DataPathTagSchema, dltag.LayerPathParameterTagSchema;

tags episodic {
tag Network with DataPath = {path = src/main/resources/training_data/ci_dataset, type = HDF5};
tag Network with LayerPathParameter = {path = src/main/resources/simplePretrained, id = simplePretrained};
}
