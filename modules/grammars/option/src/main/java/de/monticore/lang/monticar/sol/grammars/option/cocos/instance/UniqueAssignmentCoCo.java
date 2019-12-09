/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option.cocos.instance;

import de.monticore.lang.monticar.sol.grammars.option._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionASTOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.PropAssignmentSymbol;
import de.monticore.lang.monticar.sol.grammars.option.cocos.CommonOptionCoCo;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * This context condition checks whether all assignments are unique.
 */
public class UniqueAssignmentCoCo extends CommonOptionCoCo implements OptionASTOptionCoCo {
    public UniqueAssignmentCoCo() {
        super("OPT0004", "'%s' has already been assigned in option '%s'.");
    }

    @Override
    public void registerTo(OptionCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTOption node) {
        node.getOptionSymbolOpt().ifPresent(this::check);
    }

    protected void check(OptionSymbol option) {
        String name = option.getName();
        List<String> props = option.getAssignmentSymbols().stream()
                .map(PropAssignmentSymbol::getName)
                .collect(Collectors.toList());
        Set<String> uniqueProps = new HashSet<>(props);

        uniqueProps.forEach(props::remove);
        props.forEach(prop -> this.error(option, prop, name));
    }
}
