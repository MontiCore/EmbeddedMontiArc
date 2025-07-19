/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option.cocos.instance;

import de.monticore.lang.monticar.sol.grammars.option._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionASTOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.PropAssignmentSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.PropDeclarationSymbol;
import de.monticore.lang.monticar.sol.grammars.option.cocos.CommonOptionCoCo;

import java.util.List;
import java.util.stream.Collectors;

/**
 * This context condition checks whether all required assignments are present.
 */
public class RequiredAssignmentCoCo extends CommonOptionCoCo implements OptionASTOptionCoCo {
    public RequiredAssignmentCoCo() {
        super("OPT0002", "'%s' is required in option '%s'.");
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
        option.getTypeSymbol().ifPresent(type -> this.check(option, type));
    }

    protected void check(OptionSymbol option, OptionTypeSymbol type) {
        String name = option.getName();
        List<String> includedProps = option.getAssignmentSymbols().stream()
                .map(PropAssignmentSymbol::getName)
                .collect(Collectors.toList());
        List<String> requiredProps = type.getRequiredDeclarationSymbols().stream()
                .map(PropDeclarationSymbol::getName)
                .collect(Collectors.toList());

        requiredProps.removeAll(includedProps);
        requiredProps.forEach(prop -> this.error(option, prop, name));
    }
}
