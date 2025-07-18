/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ProductSymbolReference;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbolReference;
import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTConfigurationType;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTConfigurationTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol;

public class ExistingReferenceCoCo extends CommonIDECoCo implements IDEASTConfigurationTypeCoCo {
    public ExistingReferenceCoCo() {
        super("IDE0017", "Some of the %s could not be located.");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTConfigurationType node) {
        node.getConfigurationTypeSymbolOpt().ifPresent(this::check);
    }

    protected void check(ConfigurationTypeSymbol type) {
        type.getTools().forEach(tool -> this.check(type, tool));
        type.getProducts().forEach(product -> this.check(type, product));
    }

    protected void check(ConfigurationTypeSymbol type, ToolSymbolReference reference) {
        if (!reference.existsReferencedSymbol()) this.error(type, "tools");
    }

    protected void check(ConfigurationTypeSymbol type, ProductSymbolReference reference) {
        if (!reference.existsReferencedSymbol()) this.error(type, "products");
    }
}
