/* (c) https://github.com/MontiCore/monticore */
package tagging.artifacttagging;
conforms to artifacttag.DatasetArtifactTagSchema,
            artifacttag.LayerArtifactParameterTagSchema;

tags AlexNet {
    // DatasetArtifact Tags
    tag Alexnet with DatasetArtifact = {artifact = com.emadl.dataset:mnist:2, type = HDF5};
    tag Parent.a1 with DatasetArtifact = {artifact = com.emadl.dataset:sst2:40, type = LMDB};
    tag Parent.a2 with DatasetArtifact = {artifact = com.monticore.lang.monticar:imdb:2, type = HDF5};

    // LayerArtifactParameter Tags
    tag Alexnet with LayerArtifactParameter = {artifact = com.emadl.pretrained-model:bert-small:2, id = bert-small};
    tag Parent.a1 with LayerArtifactParameter = {artifact = com.emadl.pretrained-model:bert-large:1, id = bert};
}
