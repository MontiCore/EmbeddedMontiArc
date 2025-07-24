/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms;

import de.monticore.lang.monticar.clustering.ClusteringAlgorithm;

public class UnknownAlgorithmCliParameters extends AlgorithmCliParameters {

    public UnknownAlgorithmCliParameters() {
    }

    @Override
    public String getName() {
        return TYPE_UNKOWN;
    }

    @Override
    public ClusteringAlgorithm asClusteringAlgorithm() {
        return null;
    }

    @Override
    public Object[] asAlgorithmArgs() {
        return null;
    }

    @Override
    public boolean isValid() {
        return false;
    }
}
