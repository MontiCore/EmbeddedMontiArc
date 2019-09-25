/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTExpose;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentASTExposeCoCo;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.mcliterals._ast.ASTNatLiteral;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This context condition checks whether a given port lies within the valid range.
 */
public class ValidatePortCoCo implements EnvironmentCoCo, EnvironmentASTExposeCoCo {
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
    public void check(ASTExpose node) {
        ASTNatLiteral portNode = node.getPort();
        int port = portNode.getValue();

        if (port < 1024 || port > 65535) Log.warn(this.getErrorMessage(port), portNode.get_SourcePositionStart());
    }
}
