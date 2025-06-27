/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms;

import de.monticore.lang.monticar.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.clustering.algorithms.AffinityPropagationAlgorithm;

public class AffinityPropagationCliParameters extends AlgorithmCliParameters{

    public AffinityPropagationCliParameters() {
    }

    @Override
    public String getName() {
        return TYPE_AFFINITY_PROPAGATION;
    }

    @Override
    public ClusteringAlgorithm asClusteringAlgorithm() {
        return new AffinityPropagationAlgorithm();
    }

    @Override
    public Object[] asAlgorithmArgs(){
        return new Object[]{};
    }

    @Override
    public boolean isValid() {
        return true;
    }

    @Override
    public String toString() {
        return "AffinityPropagation";
    }
}
