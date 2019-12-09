/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option.cocos.instance;

import de.monticore.lang.monticar.sol.grammars.option._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionASTOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionTypeSymbolReference;
import de.monticore.lang.monticar.sol.grammars.option.cocos.CommonOptionCoCo;

/**
 * This context condition checks whether the type of an option actually exists.
 */
public class ExistingTypeCoCo extends CommonOptionCoCo implements OptionASTOptionCoCo {
    public ExistingTypeCoCo() {
        super("OPT0007", "Type of '%s' does not exist.");
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
        symbol.getType().ifPresent(reference -> this.check(symbol, reference));
    }

    protected void check(OptionSymbol symbol, OptionTypeSymbolReference reference) {
        if (!reference.existsReferencedSymbol()) this.error(symbol, symbol.getName());
    }
}
