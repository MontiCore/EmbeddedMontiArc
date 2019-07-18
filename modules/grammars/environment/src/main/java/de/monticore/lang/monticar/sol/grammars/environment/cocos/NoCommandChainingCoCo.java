/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTRunInstruction;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentASTRunInstructionCoCo;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.function.Predicate;

public class NoCommandChainingCoCo implements EnvironmentCoCo, EnvironmentASTRunInstructionCoCo {
    @Override
    public String getErrorCode() {
        return "ENV0003";
    }

    @Override
    public String getErrorMessage(Object ...parameters) {
        return String.format("%s RUN should have at most one command.", this.getErrorCode());
    }

    @Override
    public void registerTo(EnvironmentCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTRunInstruction node) {
        boolean violatesCommand = node.isPresentCommand() && this.violatesCommand(node.getCommand());
        boolean violatesParameters = node.isPresentExecutable() && this.violatesParameters(node.getParameterList());

        if (violatesCommand || violatesParameters) Log.warn(this.getErrorMessage(), node.get_SourcePositionStart());
    }

    protected boolean violatesCommand(ASTStringLiteral command) {
        return command.getValue().contains("&&");
    }

    protected boolean violatesParameters(List<ASTStringLiteral> parameters) {
        return parameters.stream().map(ASTStringLiteral::getValue).anyMatch(Predicate.isEqual("&&"));
    }
}
