/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTWorkDirInstruction;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentASTWorkDirInstructionCoCo;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class NoRelativeWorkDirCoCo implements EnvironmentCoCo, EnvironmentASTWorkDirInstructionCoCo {
    @Override
    public String getErrorCode() {
        return "ENV0006";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s Working Directory '%s' should be absolute.", parameterList.toArray());
    }

    @Override
    public void registerTo(EnvironmentCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTWorkDirInstruction node) {
        ASTStringLiteral workDirNode = node.getDirectory();
        String workDir = workDirNode.getValue();

        if (!workDir.startsWith("/")) Log.warn(this.getErrorMessage(workDir), workDirNode.get_SourcePositionStart());
    }
}
