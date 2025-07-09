/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

import de.monticore.symboltable.CommonSymbol;

public class EntrySymbol extends CommonSymbol {

    public static final EntryKind KIND = new EntryKind();
    private ValueSymbol value;

    public EntrySymbol(String name) {
        super(name, KIND);
    }

    public ValueSymbol getValue() {
        return value;
    }

    public void setValue(ValueSymbol value) {
        this.value = value;
    }
}
