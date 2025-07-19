/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.*;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentASTDockerfileCoCo;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.environment._visitor.EnvironmentVisitor;

/**
 * This context condition checks whether a blocked instruction is used in a Component Dockerfile.
 */
public class BlockedInstructionCoCo extends CommonEnvironmentCoCo implements EnvironmentASTDockerfileCoCo, EnvironmentVisitor {
    public BlockedInstructionCoCo() {
        super("ENV0007", "'%s' is not allowed in a Component Dockerfile.");
    }

    @Override
    public void registerTo(EnvironmentCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTDockerfile node) {
        this.handle(node);
    }

    @Override
    public void handle(ASTDockerfile node) {
        getRealThis().visit(node);

        if (node.isComponent()) getRealThis().traverse(node);

        getRealThis().endVisit(node);
    }

    @Override
    public void handle(ASTImport node) {
        this.error(node, "IMPORT");
    }

    @Override
    public void handle(ASTFrom node) {
        this.fail(node);
    }

    @Override
    public void handle(ASTCMD node) {
        this.fail(node);
    }

    @Override
    public void handle(ASTAdd node) {
        this.fail(node);
    }

    @Override
    public void handle(ASTCopy node) {
        this.fail(node);
    }

    @Override
    public void handle(ASTEntryPoint node) {
        this.fail(node);
    }

    @Override
    public void handle(ASTUser node) {
        this.fail(node);
    }

    @Override
    public void handle(ASTOnBuild node) {
        this.fail(node);
    }

    @Override
    public void handle(ASTStopSignal node) {
        this.fail(node);
    }

    @Override
    public void handle(ASTHealthCheck node) {
        this.fail(node);
    }

    protected void fail(ASTInstruction node) {
        this.error(node, node.getType());
    }
}
