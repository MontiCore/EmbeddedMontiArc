/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option.cocos.type;

import de.monticore.lang.monticar.sol.grammars.option._ast.ASTOptionType;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionASTOptionTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.PropDeclarationSymbol;
import de.monticore.lang.monticar.sol.grammars.option.cocos.CommonOptionCoCo;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class UniqueDeclarationCoCo extends CommonOptionCoCo implements OptionASTOptionTypeCoCo {
    public UniqueDeclarationCoCo() {
        super("OPT0006", "'%s' has already been declared in '%s'.");
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
        List<String> props = type.getDeclarationSymbols().stream()
                .map(PropDeclarationSymbol::getName)
                .collect(Collectors.toList());
        Set<String> uniqueProps = new HashSet<>(props);

        uniqueProps.forEach(props::remove);
        props.forEach(prop -> this.error(type, prop, name));
    }
}
