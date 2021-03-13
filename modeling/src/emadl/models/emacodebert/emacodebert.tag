/* (c) https://github.com/MontiCore/monticore */
package emacodebert;
conforms to dltag.DataPathTagSchema, dltag.LayerPathParameterTagSchema;

tags episodic {
tag Network with DataPath = {path = resources/training_data/128/set0, type = HDF5};
tag Network with LayerPathParameter = {path = ../packaging/resources/pretrained_codebert, id = codebert};
}
