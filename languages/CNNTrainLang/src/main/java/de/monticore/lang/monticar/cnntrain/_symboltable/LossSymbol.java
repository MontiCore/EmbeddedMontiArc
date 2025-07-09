/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

import java.util.HashMap;
import java.util.Map;

public class LossSymbol extends de.monticore.symboltable.CommonSymbol {

    private Map<String, LossParamSymbol> lossParamMap = new HashMap<>();
    public static final LossSymbolKind KIND = LossSymbolKind.INSTANCE;

    public LossSymbol(String name) {
        super(name, KIND);
    }

    public Map<String, LossParamSymbol> getLossParamMap() {
        return lossParamMap;
    }

    public void setLossParamMap(Map<String, LossParamSymbol> lossParamMap) {
        this.lossParamMap = lossParamMap;
    }
}
