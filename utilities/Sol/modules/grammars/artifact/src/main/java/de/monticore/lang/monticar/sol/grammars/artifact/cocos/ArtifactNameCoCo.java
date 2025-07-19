/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact.cocos;

import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTArtifact;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTProduct;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTTool;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactASTArtifactCoCo;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactASTProductCoCo;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactASTToolCoCo;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactCoCoChecker;
import de.monticore.symboltable.Symbol;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * This context condition checks whether the format of a resource's name is valid.
 */
public class ArtifactNameCoCo extends CommonArtifactCoCo implements ArtifactASTArtifactCoCo, ArtifactASTToolCoCo,
        ArtifactASTProductCoCo {
    protected final Pattern pattern;

    public ArtifactNameCoCo() {
        super("SYS0002", "Artifact '%s' should start with a Capital letter.");

        this.pattern = Pattern.compile("[A-Z][[A-Z][a-z][0-9]_]*");
    }

    @Override
    public void registerTo(ArtifactCoCoChecker checker) {
        checker.addCoCo((ArtifactASTArtifactCoCo)this);
        checker.addCoCo((ArtifactASTToolCoCo)this);
        checker.addCoCo((ArtifactASTProductCoCo)this);
    }

    @Override
    public void check(ASTArtifact node) {
        node.getArtifactSymbolOpt().ifPresent(this::check);
    }

    @Override
    public void check(ASTProduct node) {
        node.getProductSymbolOpt().ifPresent(this::check);
    }

    @Override
    public void check(ASTTool node) {
        node.getToolSymbolOpt().ifPresent(this::check);
    }

    protected void check(Symbol symbol) {
        String name = symbol.getName();
        Matcher matcher = this.pattern.matcher(name);

        if (!matcher.matches()) this.warn(symbol, name);
    }
}
