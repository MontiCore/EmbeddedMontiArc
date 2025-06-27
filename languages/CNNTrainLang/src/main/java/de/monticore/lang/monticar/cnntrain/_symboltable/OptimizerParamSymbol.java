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

public class OptimizerParamSymbol extends CommonSymbol {

    public static final EntryKind KIND = new EntryKind();

    private OptimizerParamValueSymbol value;

    public OptimizerParamSymbol() {
        super("", KIND);
    }

    public OptimizerParamSymbol(String name, SymbolKind kind) {
        super(name, kind);
    }

    public OptimizerParamValueSymbol getValue() {
        return value;
    }

    public void setValue(OptimizerParamValueSymbol value) {
        this.value = value;
    }

    public String toString(){
        return getValue().toString();
    }
}
