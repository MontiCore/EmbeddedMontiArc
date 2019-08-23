/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.*;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentASTDockerfileCoCo;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.environment._visitor.EnvironmentVisitor;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This context condition checks whether a blocked instruction is used in a Component Dockerfile.
 */
public class BlockedInstructionCoCo implements EnvironmentASTDockerfileCoCo, EnvironmentCoCo, EnvironmentVisitor {
    @Override
    public String getErrorCode() {
        return "ENV0007";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s '%s' is not allowed in Component Dockerfile.", parameterList.toArray());
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
        Log.warn(this.getErrorMessage(node.getType()), node.get_SourcePositionStart());
    }
}
