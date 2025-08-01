/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option._symboltable.assignment;

import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithValue;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.PropAssignmentSymbol;

import java.util.Optional;

public class LiteralAssignmentSymbol extends PropAssignmentSymbol implements SymbolWithValue {
    protected Object value;

    public LiteralAssignmentSymbol(String name) {
        super(name);
    }

    @Override
    public void setValue(Object value) {
        this.value = value;
    }

    @Override
    public Optional<Object> getValue() {
        return Optional.ofNullable(this.value);
    }
}
