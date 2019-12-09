/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTExpose;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentASTExposeCoCo;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.mcliterals._ast.ASTNatLiteral;

/**
 * This context condition checks whether a given port lies within the valid range.
 */
public class ValidatePortCoCo extends CommonEnvironmentCoCo implements EnvironmentASTExposeCoCo {
    public ValidatePortCoCo() {
        super("ENV0005", "Port '%d' should be between 1024 and 65535.");
    }

    @Override
    public void registerTo(EnvironmentCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTExpose node) {
        ASTNatLiteral portNode = node.getPort();
        int port = portNode.getValue();

        if (port < 1024 || port > 65535) this.error(node, port);
    }
}
