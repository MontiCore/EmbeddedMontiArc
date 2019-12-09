/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTConfiguration;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTConfigurationCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.OptionFillSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.OptionInheritSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class OptionsCoCo extends CommonIDECoCo implements IDEASTConfigurationCoCo {
    public OptionsCoCo() {
        super("IDE0007", "Option '%s' of configuration '%s' must either be filled or inherited.");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTConfiguration node) {
        node.getConfigurationSymbolOpt().ifPresent(this::check);
    }

    protected void check(ConfigurationSymbol configuration) {
        configuration.getTypeSymbol().ifPresent(type -> this.check(configuration, type));
    }

    protected void check(ConfigurationSymbol configuration, ConfigurationTypeSymbol type) {
        List<String> neededOptions = type.getOptionSymbols().stream()
                .map(OptionSymbol::getName)
                .collect(Collectors.toList());
        List<String> fills = configuration.getOptionFillSymbols().stream()
                .map(OptionFillSymbol::getName)
                .collect(Collectors.toList());
        List<String> inherits = configuration.getOptionInheritSymbols().stream()
                .map(OptionInheritSymbol::getName)
                .collect(Collectors.toList());

        this.check(configuration, neededOptions, fills, inherits);
    }

    protected void check(ConfigurationSymbol configuration, List<String> neededOptions, List<String> fills,
                         List<String> inherits) {
        String name = configuration.getName();
        List<String> setOptions = new ArrayList<>();

        setOptions.addAll(fills);
        setOptions.addAll(inherits);

        setOptions.forEach(neededOptions::remove);
        neededOptions.forEach(option -> this.error(configuration, option, name));
    }
}
