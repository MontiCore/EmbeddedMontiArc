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

import java.util.Optional;

public class ValueSymbol extends CommonSymbol {

    public static final ValueKind KIND = new ValueKind();

    private Object value;

    public ValueSymbol() {
        super("", KIND);
    }

    public ValueSymbol(String name, SymbolKind kind) {
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
