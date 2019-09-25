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

public class LossParamSymbol extends CommonSymbol {

    public static final EntryKind KIND = new EntryKind();

    private LossParamValueSymbol value;

    public LossParamSymbol() {
        super("", KIND);
    }

    public LossParamSymbol(String name, SymbolKind kind) {
        super(name, kind);
    }

    public LossParamValueSymbol getValue() {
        return value;
    }

    public void setValue(LossParamValueSymbol value) {
        this.value = value;
    }

    public String toString(){
        return getValue().toString();
    }
}
