/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable.fill;

import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithValue;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.OptionFillSymbol;

import java.util.Optional;

public class LiteralFillSymbol extends OptionFillSymbol implements SymbolWithValue {
    protected Object value;

    public LiteralFillSymbol(String name) {
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
