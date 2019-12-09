/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option.cocos.instance;

import de.monticore.lang.monticar.sol.grammars.option._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionASTOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option.cocos.CommonOptionCoCo;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * This context condition checks whether all sub-options are unique.
 */
public class UniqueOptionCoCo extends CommonOptionCoCo implements OptionASTOptionCoCo {
    public UniqueOptionCoCo() {
        super("OPT0005", "There is already a sub-option '%s' in option '%s'.");
    }

    @Override
    public void registerTo(OptionCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTOption node) {
        node.getOptionSymbolOpt().ifPresent(this::check);
    }

    protected void check(OptionSymbol symbol) {
        Set<String> names = symbol.getOptionSymbols().stream()
                .map(OptionSymbol::getName)
                .collect(Collectors.toSet());
        List<String> workingList = symbol.getOptionSymbols().stream()
                .map(OptionSymbol::getName)
                .collect(Collectors.toList());

        names.forEach(workingList::remove);
        workingList.forEach(name -> this.error(symbol, name, symbol.getName()));
    }
}
