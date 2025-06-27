/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms.dynamic;

import de.monticore.lang.monticar.clustering.cli.algorithms.AlgorithmCliParameters;
import de.monticore.lang.monticar.clustering.cli.algorithms.UnknownAlgorithmCliParameters;

import java.util.Arrays;
import java.util.List;

public class DynamicUnknownAlgorithmCliParameters extends DynamicAlgorithmCliParameters {

    public DynamicUnknownAlgorithmCliParameters() {
    }

    @Override
    public List<AlgorithmCliParameters> getAll() {
        return Arrays.asList(new UnknownAlgorithmCliParameters());
    }

    @Override
    public boolean isValid() {
        return false;
    }

    @Override
    public String getName() {
        return AlgorithmCliParameters.TYPE_UNKOWN;
    }


}
