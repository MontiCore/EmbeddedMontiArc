/* (c) https://github.com/MontiCore/monticore */
package gCN;
conforms to dltag.DataPathTagSchema;

tags NeuronalNetwork {
 tag GatNetwork with DataPath = {path = src/main/resources/training_data_dgl_cora, type = HDF5};
}