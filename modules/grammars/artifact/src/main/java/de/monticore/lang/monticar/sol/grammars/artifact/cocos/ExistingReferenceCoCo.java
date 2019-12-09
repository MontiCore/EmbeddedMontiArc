/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileSymbolReference;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbolReference;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTProduct;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTTool;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactASTProductCoCo;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactASTToolCoCo;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.SymbolWithEnvironment;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.SymbolWithLanguages;

/**
 * This context condition checks whether the references language / environment actually exists.
 */
public class ExistingReferenceCoCo extends CommonArtifactCoCo implements ArtifactASTToolCoCo, ArtifactASTProductCoCo {
    public ExistingReferenceCoCo() {
        super("SYS0005", "%s could not be resolved for '%s'.");
    }

    @Override
    public void registerTo(ArtifactCoCoChecker checker) {
        checker.addCoCo((ArtifactASTToolCoCo) this);
        checker.addCoCo((ArtifactASTProductCoCo) this);
    }

    @Override
    public void check(ASTProduct node) {
        node.getProductSymbolOpt().ifPresent(symbol -> {
            this.check((SymbolWithEnvironment) symbol);
            this.check((SymbolWithLanguages) symbol);
        });
    }

    @Override
    public void check(ASTTool node) {
        node.getToolSymbolOpt().ifPresent(symbol -> {
            this.check((SymbolWithEnvironment) symbol);
            this.check((SymbolWithLanguages) symbol);
        });
    }

    protected void check(SymbolWithEnvironment symbol) {
        symbol.getEnvironment().ifPresent(reference -> this.check(symbol, reference));
    }

    protected void check(SymbolWithEnvironment symbol, DockerfileSymbolReference reference) {
        if (!reference.existsReferencedSymbol()) this.error(symbol, "Environment", symbol.getName());
    }

    protected void check(SymbolWithLanguages symbol) {
        symbol.getLanguages().forEach(reference -> this.check(symbol, reference));
    }

    protected void check(SymbolWithLanguages symbol, LanguageSymbolReference reference) {
        if (!reference.existsReferencedSymbol()) this.error(symbol, "Language", symbol.getName());
    }
}
