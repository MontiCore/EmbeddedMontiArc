/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact.cocos;

import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTProduct;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTTool;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactASTProductCoCo;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactASTToolCoCo;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactSymbol;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.SymbolWithArtifacts;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvedSeveralEntriesException;

import java.util.*;
import java.util.stream.Collectors;

/**
 * This context condition checks whether no artifact has been declared multiple times.
 */
public class UniqueArtifactCoCo extends CommonArtifactCoCo implements ArtifactASTToolCoCo, ArtifactASTProductCoCo {
    public UniqueArtifactCoCo() {
        super("SYS0003", "There is already an artifact '%s' in '%s'.");
    }

    @Override
    public void registerTo(ArtifactCoCoChecker checker) {
        checker.addCoCo((ArtifactASTToolCoCo) this);
        checker.addCoCo((ArtifactASTProductCoCo) this);
    }

    @Override
    public void check(ASTProduct node) {
        node.getProductSymbolOpt().ifPresent(this::check);
    }

    @Override
    public void check(ASTTool node) {
        node.getToolSymbolOpt().ifPresent(this::check);
    }

    protected void check(SymbolWithArtifacts symbol) {
        try {
            this.doCheck(symbol);
        } catch(ResolvedSeveralEntriesException exception) {
            Deque<Symbol> symbols = new ArrayDeque<>(exception.getSymbols());
            Symbol artifact = symbols.getLast();

            this.error(symbol, artifact.getName(), symbol.getName());
        }
    }

    protected void doCheck(SymbolWithArtifacts symbol) {
        String name = symbol.getName();
        List<String> artifacts = symbol.getArtifactSymbols().stream()
                .map(ArtifactSymbol::getName)
                .collect(Collectors.toList());
        Set<String> uniqueArtifacts = new HashSet<>(artifacts);

        uniqueArtifacts.forEach(artifacts::remove);
        artifacts.forEach(artifact -> this.error(symbol, artifact, name));
    }
}
