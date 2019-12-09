/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable.fill;

import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithValueList;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.OptionFillSymbol;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class LiteralListFillSymbol extends OptionFillSymbol implements SymbolWithValueList {
    protected final List<Object> values;

    protected String type;

    public LiteralListFillSymbol(String name) {
        super(name);

        this.values = new ArrayList<>();
    }

    @Override
    public void addValue(Object value) {
        this.values.add(value);
    }

    @Override
    public void removeValue(Object value) {
        this.values.remove(value);
    }

    @Override
    public void setValues(List<Object> values) {
        this.values.clear();
        this.values.addAll(values);
    }

    @Override
    public void addValues(List<Object> values) {
        this.values.addAll(values);
    }

    @Override
    public List<Object> getValues() {
        return Collections.unmodifiableList(this.values);
    }

    @Override
    public void setType(String type) {
        this.type = type;
    }

    @Override
    public Optional<String> getType() {
        return Optional.ofNullable(this.type);
    }
}
