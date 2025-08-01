/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithType;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithValue;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithValueList;
import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTConfiguration;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTConfigurationCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.OptionFillSymbol;

import java.util.Map;

public class OptionFillTypeCoCo extends OptionCommonCoCo implements IDEASTConfigurationCoCo {
    public OptionFillTypeCoCo() {
        super("IDE0003", "Fill '%s' is not of the expected type '%s'.");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTConfiguration node) {
        node.getConfigurationSymbolOpt().ifPresent(this::check);
    }

    protected void check(ConfigurationSymbol symbol) {
        Map<String, String> typeMap = this.getTypeMapOf(symbol);

        symbol.getOptionFillSymbols().forEach(fill -> this.check(fill, typeMap));
    }

    protected void check(OptionFillSymbol symbol, Map<String, String> typeMap) {
        symbol.asLiteralFill().ifPresent(literal -> this.check((SymbolWithValue) literal, typeMap));
        symbol.asLiteralListFill().ifPresent(list -> this.check((SymbolWithValueList) list, typeMap));
    }

    protected void check(SymbolWithValue symbol, Map<String, String> typeMap) {
        symbol.getType().ifPresent(actualType -> this.check(symbol, typeMap, actualType));
    }

    protected void check(SymbolWithValueList symbol, Map<String, String> typeMap) {
        symbol.getType().ifPresent(actualType -> this.check(symbol, typeMap, actualType));
    }

    protected void check(SymbolWithType symbol, Map<String, String> typeMap, String actualType) {
        String name = symbol.getName();
        String expectedType = typeMap.getOrDefault(name, "UNKNOWN_TYPE");

        if (!expectedType.equals(actualType)) this.error(symbol, name, expectedType);
    }
}
