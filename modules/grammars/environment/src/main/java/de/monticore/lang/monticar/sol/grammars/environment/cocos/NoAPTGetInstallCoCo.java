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

public class NoAPTGetInstallCoCo implements EnvironmentCoCo, EnvironmentASTRunInstructionCoCo {
    @Override
    public String getErrorCode() {
        return "ENV0001";
    }

    @Override
    public String getErrorMessage(Object ...parameters) {
        return String.format("%s Please use 'INSTALL <package> [,<package>]' instead of 'RUN apt-get install'.", this.getErrorCode());
    }

    @Override
    public void registerTo(EnvironmentCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTRunInstruction node) {
        boolean commandViolation = node.isPresentCommand() && this.violatesCommand(node.getCommand());
        boolean executableViolation = node.isPresentExecutable()
                && this.violatesExecutableParameters(node.getExecutable(), node.getParameterList());

        if (commandViolation || executableViolation) Log.warn(this.getErrorMessage(), node.get_SourcePositionStart());
    }

    protected boolean violatesCommand(ASTStringLiteral command) {
        String[] commandParts = command.getValue().split(" ");

        return commandParts.length > 1
                && commandParts[0].equals("apt-get")
                && commandParts[1].equals("install");
    }

    protected boolean violatesExecutableParameters(ASTStringLiteral executable, List<ASTStringLiteral> parameters) {
        return executable.getValue().equals("apt-get")
                && parameters.size() > 0
                && parameters.get(0).getValue().equals("install");
    }
}
