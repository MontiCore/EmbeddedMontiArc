/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain.annotations;

import java.util.List;
import java.util.Optional;

/**
 *
 */
public interface RewardFunctionParameter {
    List<String> getInputNames();
    List<String> getOutputNames();
    Optional<String> getTypeOfInputPort(String portName);
    Optional<String> getTypeOfOutputPort(String portName);
    Optional<List<Integer>> getInputPortDimensionOfPort(String portName);
    Optional<List<Integer>> getOutputPortDimensionOfPort(String portName);
}
