/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

import de.monticore.lang.monticar.cnntrain.annotations.Range;
import de.monticore.symboltable.SymbolKind;

import java.util.List;
import java.util.Map;

public abstract class NNArchitectureSymbol extends de.monticore.symboltable.CommonSymbol {
    public static final NNArchitectureSymbolKind KIND = NNArchitectureSymbolKind.INSTANCE;

    public NNArchitectureSymbol(String name) {
        super(name, KIND);
    }

    abstract public List<String> getInputs();
    abstract public List<String> getOutputs();
    abstract public Map<String, List<Integer>> getDimensions();
    abstract public Map<String, Range> getRanges();
    abstract public Map<String, String> getTypes();
}