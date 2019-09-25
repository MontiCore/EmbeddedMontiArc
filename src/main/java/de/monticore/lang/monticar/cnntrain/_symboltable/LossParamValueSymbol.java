/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.SymbolKind;

public class LossParamValueSymbol extends CommonSymbol {

    public static final LossParamValueSymbolKind KIND = new LossParamValueSymbolKind();

    private Object value;

    public LossParamValueSymbol() {
        super("", KIND);
    }

    public LossParamValueSymbol(String name, SymbolKind kind) {
        super(name, kind);
    }

    public Object getValue() {
        return value;
    }

    public void setValue(Object value) {
        this.value = value;
    }

    public String toString(){
        return getValue().toString();
    }
}
