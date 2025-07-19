/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.*;

import java.util.List;
import java.util.Optional;

/**
 * ATTENTION READER: This interface and its corresponding implementation are necessary because the Freemarker
 * version of MontiCore does not support the default methods of Java 8.
 */
public interface CommonMethodDelegator {
    Optional<String> getPath(SymbolWithPath symbol);
    Optional<CommonLiterals> getOrigin(SymbolWithProjectPath symbol);
    Optional<String> getType(SymbolWithType symbol);
    Optional<Object> getValue(SymbolWithValue symbol);
    List<Object> getValues(SymbolWithValueList symbol);
    boolean isStringList(SymbolWithValueList symbol);
}
