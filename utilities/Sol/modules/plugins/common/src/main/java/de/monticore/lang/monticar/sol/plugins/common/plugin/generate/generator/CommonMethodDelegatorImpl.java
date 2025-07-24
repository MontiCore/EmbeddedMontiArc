/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.*;

import java.util.List;
import java.util.Optional;

@Singleton
public class CommonMethodDelegatorImpl implements CommonMethodDelegator {
    @Override
    public Optional<String> getPath(SymbolWithPath symbol) {
        return symbol.getPath();
    }

    @Override
    public Optional<CommonLiterals> getOrigin(SymbolWithProjectPath symbol) {
        return symbol.getOrigin();
    }

    @Override
    public Optional<String> getType(SymbolWithType symbol) {
        return symbol.getType();
    }

    @Override
    public Optional<Object> getValue(SymbolWithValue symbol) {
        return symbol.getValue();
    }

    @Override
    public List<Object> getValues(SymbolWithValueList symbol) {
        return symbol.getValues();
    }

    @Override
    public boolean isStringList(SymbolWithValueList symbol) {
        return symbol.isStringList();
    }
}
