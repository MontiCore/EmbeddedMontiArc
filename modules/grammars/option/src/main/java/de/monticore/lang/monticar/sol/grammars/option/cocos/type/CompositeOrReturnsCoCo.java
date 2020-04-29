/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option.cocos.type;

import de.monticore.lang.monticar.sol.grammars.option._ast.ASTOptionType;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionASTOptionTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.option.cocos.CommonOptionCoCo;

import java.util.Optional;

public class CompositeOrReturnsCoCo extends CommonOptionCoCo implements OptionASTOptionTypeCoCo {
    public CompositeOrReturnsCoCo() {
        super("OPT0008", "Option type '%s' must either be a composite or have a returns statement.");
    }

    @Override
    public void registerTo(OptionCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTOptionType node) {
        node.getOptionTypeSymbolOpt().ifPresent(this::check);
    }

    protected void check(OptionTypeSymbol type) {
        String name = type.getName();
        Optional<String> returnType = type.getReturnType();
        boolean condition = (type.isComposite() && returnType.isPresent()) ||
                (!type.isComposite() && !returnType.isPresent());

        if (condition) this.error(type, name);
    }
}
