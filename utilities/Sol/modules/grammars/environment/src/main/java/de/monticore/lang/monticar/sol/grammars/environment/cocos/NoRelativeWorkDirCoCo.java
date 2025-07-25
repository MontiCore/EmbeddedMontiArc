/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTWorkDir;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentASTWorkDirCoCo;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.mcliterals._ast.ASTStringLiteral;

/**
 * This context condition checks whether a relative path has been used to in the WORKDIR instruction.
 */
public class NoRelativeWorkDirCoCo extends CommonEnvironmentCoCo implements EnvironmentASTWorkDirCoCo {
    public NoRelativeWorkDirCoCo() {
        super("ENV0006", "Working Directory '%s' should be absolute.");
    }

    @Override
    public void registerTo(EnvironmentCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTWorkDir node) {
        ASTStringLiteral workDirNode = node.getDirectory();
        String workDir = workDirNode.getValue();

        if (!workDir.startsWith("/")) this.error(node, workDir);
    }
}
