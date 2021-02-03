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

public class InitializerParamSymbol extends CommonSymbol {

    public static final EntryKind KIND = new EntryKind();

    private InitializerParamValueSymbol value;

    public InitializerParamSymbol() {
        super("", KIND);
    }

    public InitializerParamSymbol(String name, SymbolKind kind) {
        super(name, kind);
    }

    public InitializerParamValueSymbol getValue() {
        return value;
    }

    public void setValue(InitializerParamValueSymbol value) {
        this.value = value;
    }

    public String toString(){
        return getValue().toString();
    }
}
