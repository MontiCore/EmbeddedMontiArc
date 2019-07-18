/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTExposeInstruction;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentASTExposeInstructionCoCo;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.mcliterals._ast.ASTNatLiteral;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ValidatePortCoCo implements EnvironmentCoCo, EnvironmentASTExposeInstructionCoCo {
    @Override
    public String getErrorCode() {
        return "ENV0005";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s Port '%d' should be between 1024 and 65535.", parameterList.toArray());
    }

    @Override
    public void registerTo(EnvironmentCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTExposeInstruction node) {
        ASTNatLiteral portNode = node.getPort();
        int port = portNode.getValue();

        if (port < 1024 || port > 65535) Log.warn(this.getErrorMessage(port), portNode.get_SourcePositionStart());
    }
}
