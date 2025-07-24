/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact.cocos;

import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.sol.grammars.artifact._ast.*;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactASTArtifactCoCo;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactASTProductCoCo;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactASTToolCoCo;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.artifact._visitor.ArtifactVisitor;

import java.util.HashMap;
import java.util.Map;

/**
 * This context condition checks whether no attribute has been declared multiple times.
 */
public class UniqueAttributeCoCo extends CommonArtifactCoCo
        implements ArtifactASTToolCoCo, ArtifactASTProductCoCo, ArtifactASTArtifactCoCo, ArtifactVisitor {
    protected final Map<String, Integer> counters;

    protected ASTArtifactNode target;

    public UniqueAttributeCoCo() {
        super("SYS0004", "There is already an attribute '%s' in '%s'.");

        this.counters = new HashMap<>();
    }

    @Override
    public void registerTo(ArtifactCoCoChecker checker) {
        checker.addCoCo((ArtifactASTArtifactCoCo) this);
        checker.addCoCo((ArtifactASTProductCoCo) this);
        checker.addCoCo((ArtifactASTToolCoCo) this);
    }

    protected void addEncounter(String attribute) {
        this.counters.put(attribute, 1 + this.counters.getOrDefault(attribute, 0));
    }

    protected void countEncounters(ASTNode node, String name) {
        this.counters.forEach((attribute, value) -> {
            if (value > 1) this.warn(node, attribute, name);
        });
    }

    @Override
    public void check(ASTArtifact node) {
        this.check(node, node.getName());
    }

    @Override
    public void check(ASTProduct node) {
        this.check(node, node.getName());
    }

    @Override
    public void check(ASTTool node) {
        this.check(node, node.getName());
    }

    protected void check(ASTArtifactNode node, String name) {
        this.target = node;

        this.counters.clear();
        node.accept(this);
        this.countEncounters(node, name);
    }

    @Override
    public void handle(ASTArtifact node) {
        if (this.target == node) {
            this.visit(node);
            this.traverse(node);
            this.endVisit(node);
        }
    }

    @Override
    public void visit(ASTPath node) {
        this.addEncounter("path");
    }

    @Override
    public void visit(ASTCommand node) {
        if (node.isPresentPrefix()) this.addEncounter("prefix");
        else if (node.isPresentSuffix()) this.addEncounter("suffix");
        else if (node.isPresentCommand()) this.addEncounter("command");
    }

    @Override
    public void visit(ASTEnvironment node) {
        this.addEncounter("environment");
    }

    @Override
    public void visit(ASTAlias node) {
        this.addEncounter("alias");
    }

    @Override
    public void visit(ASTLanguage node) {
        this.addEncounter("language");
    }
}
