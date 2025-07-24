/* (c) https://github.com/MontiCore/monticore */
package letterpred;
//conforms to dltag.DataPathTagSchema;
conforms to artifacttag.DatasetArtifactTagSchema;

tags NeuronalNetwork {
   //tag Network with DataPath = {path = src/resources/training_data_letters, type = HDF5};
    tag Network with DatasetArtifact = {artifact = de.monticore.lang.monticar.datasets:mnist-letters:1, type = HDF5};
}

