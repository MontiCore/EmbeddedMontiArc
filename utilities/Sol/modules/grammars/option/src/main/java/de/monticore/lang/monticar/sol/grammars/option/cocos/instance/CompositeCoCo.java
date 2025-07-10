/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option.cocos.instance;

import de.monticore.lang.monticar.sol.grammars.option._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionASTOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.option.cocos.CommonOptionCoCo;

/**
 * This context condition checks whether a given option with sub-options is a composite.
 */
public class CompositeCoCo extends CommonOptionCoCo implements OptionASTOptionCoCo {
    public CompositeCoCo() {
        super("OPT0001", "Option '%s' cannot have sub-options as its type is not a composite.");
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
        if (!type.isComposite() && !option.getOptionSymbols().isEmpty()) this.error(option, option.getName());
    }
}
