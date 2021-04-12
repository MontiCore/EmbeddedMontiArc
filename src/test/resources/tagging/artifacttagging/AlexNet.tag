/* (c) https://github.com/MontiCore/monticore */
package tagging.artifacttagging;
conforms to artifacttag.DatasetArtifactTagSchema,
            artifacttag.LayerArtifactParameterTagSchema;

tags AlexNet {
    tag Alexnet with DatasetArtifact = {artifact = com.emadl.dataset:mnist:2, type = HDF5};
    tag Parent.a1 with DatasetArtifact = {artifact = com.emadl.dataset:sst2:40, type = LMDB};
    tag Parent.a2 with DatasetArtifact = {artifact = com.monticore.lang.monticar:imdb:2, type = HDF5};
}
