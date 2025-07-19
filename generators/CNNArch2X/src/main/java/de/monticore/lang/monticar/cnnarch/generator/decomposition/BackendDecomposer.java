package de.monticore.lang.monticar.cnnarch.generator.decomposition;

public interface BackendDecomposer {

    void decomposeNetwork(String modelPath, NetworkStructure composedNetworkStructure);

}
