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

public class InitializerSymbol extends de.monticore.symboltable.CommonSymbol {

    private Map<String, InitializerParamSymbol> initializerParamMap = new HashMap<>();
    public static final InitializerSymbolKind KIND = InitializerSymbolKind.INSTANCE;

    public InitializerSymbol(String name) {
        super(name, KIND);
    }

    public Map<String, InitializerParamSymbol> getInitializerParamMap() {
        return initializerParamMap;
    }

    public void setInitializerParamMap(Map<String, InitializerParamSymbol> initializerParamMap) {
        this.initializerParamMap = initializerParamMap;
    }
}
