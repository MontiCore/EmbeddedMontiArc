/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator.annotations;

import java.util.List;
import java.util.Map;

public interface NNArchitecture {

    String getName();
    List<String> getInputs();
    List<String> getOutputs();
    Map<String, List<Integer>> getDimensions();
    Map<String, Range> getRanges();
    Map<String, String> getTypes();
}