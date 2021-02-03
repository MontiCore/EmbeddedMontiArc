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

public class InitializerParamValueSymbol extends CommonSymbol {

    public static final InitializerParamValueSymbolKind KIND = new InitializerParamValueSymbolKind();

    private Object value;

    public InitializerParamValueSymbol() {
        super("", KIND);
    }

    public InitializerParamValueSymbol(String name, SymbolKind kind) {
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
