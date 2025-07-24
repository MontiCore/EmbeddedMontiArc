/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact.cocos;

import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithPath;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTArtifact;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTProduct;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTTool;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactASTArtifactCoCo;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactASTProductCoCo;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactASTToolCoCo;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactSymbol;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ProductSymbol;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbol;

import java.util.Optional;

/**
 * This context condition checks whether all model elements have the attributes that they require.
 */
public class RequiredAttributesCoCo extends CommonArtifactCoCo
        implements ArtifactASTToolCoCo, ArtifactASTProductCoCo, ArtifactASTArtifactCoCo {
    public RequiredAttributesCoCo() {
        super("SYS0001", "'%s' should have an attribute '%s'.");
    }

    @Override
    public void registerTo(ArtifactCoCoChecker checker) {
        checker.addCoCo((ArtifactASTToolCoCo)this);
        checker.addCoCo((ArtifactASTProductCoCo)this);
        checker.addCoCo((ArtifactASTArtifactCoCo)this);
    }

    @Override
    public void check(ASTArtifact node) {
        node.getArtifactSymbolOpt().ifPresent(this::check);
    }

    protected void check(ArtifactSymbol artifact) {
        this.check((SymbolWithPath)artifact);
    }

    @Override
    public void check(ASTProduct node) {
        node.getProductSymbolOpt().ifPresent(this::check);
    }

    protected void check(ProductSymbol product) {
        this.check((SymbolWithPath)product);
    }

    @Override
    public void check(ASTTool node) {
        node.getToolSymbolOpt().ifPresent(this::check);
    }

    protected void check(ToolSymbol tool) {
        String name = tool.getName();
        boolean isVirtual = tool.isVirtual();
        boolean hasCommand = tool.getCommand().isPresent();

        if (!isVirtual) this.check((SymbolWithPath)tool);
        if (isVirtual && !hasCommand) this.error(tool, name, "command");
    }

    protected void check(SymbolWithPath symbol) {
        String name = symbol.getName();
        Optional<String> path = symbol.getPath();

        if (!path.isPresent()) this.error(symbol, name, "path");
    }
}
