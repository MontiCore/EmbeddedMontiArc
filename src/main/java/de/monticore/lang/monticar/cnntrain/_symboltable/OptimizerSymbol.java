/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

import java.util.HashMap;
import java.util.Map;

public class OptimizerSymbol extends de.monticore.symboltable.CommonSymbol {

    private Map<String, OptimizerParamSymbol> optimizerParamMap = new HashMap<>();
    public static final OptimizerSymbolKind KIND = OptimizerSymbolKind.INSTANCE;

    public OptimizerSymbol(String name) {
        super(name, KIND);
    }

    public Map<String, OptimizerParamSymbol> getOptimizerParamMap() {
        return optimizerParamMap;
    }

    public void setOptimizerParamMap(Map<String, OptimizerParamSymbol> optimizerParamMap) {
        this.optimizerParamMap = optimizerParamMap;
    }
}
