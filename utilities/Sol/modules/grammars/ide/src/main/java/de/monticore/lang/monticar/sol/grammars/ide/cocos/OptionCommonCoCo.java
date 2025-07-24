/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionTypeSymbol;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class OptionCommonCoCo extends CommonIDECoCo { // TODO: Use full qualified name instead of simple name.
    protected OptionCommonCoCo(String errorCode, String schema) {
        super(errorCode, schema);
    }

    protected Map<String, String> getTypeMapOf(ConfigurationSymbol configuration) {
        return configuration.getTypeSymbol()
                .map(this::getTypeMapOf)
                .orElse(new HashMap<>());
    }

    protected Map<String, String> getTypeMapOf(ConfigurationTypeSymbol type) {
        Map<String, String> result = new HashMap<>();
        List<OptionSymbol> options = type.getOptionSymbols();

        options.forEach(option -> option.getTypeSymbol()
                .flatMap(OptionTypeSymbol::getReturnType)
                .ifPresent(returnType -> result.put(option.getName(), returnType)));

        return result;
    }
}
