/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTCommandOrSplitCommand;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTRun;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTSplitCommand;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentASTRunCoCo;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.environment._visitor.EnvironmentVisitor;
import de.monticore.mcliterals._ast.ASTStringLiteral;

import java.util.List;

/**
 *  This context condition checks whether RUN "apt-get install" is used instead of INSTALL.
 */
public class NoAPTGetInstallCoCo extends CommonEnvironmentCoCo implements EnvironmentASTRunCoCo, EnvironmentVisitor {
    public NoAPTGetInstallCoCo() {
        super("ENV0001", "Please use 'INSTALL <package> [,<package>]' instead of 'RUN apt-get install'.");
    }

    @Override
    public void registerTo(EnvironmentCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTRun node) {
        node.accept(getRealThis());
    }

    @Override
    public void visit(ASTCommandOrSplitCommand node) {
        node.getCommandOpt().ifPresent(command -> {
            if (this.violatesCommand(command)) this.error(node);
        });
    }

    @Override
    public void visit(ASTSplitCommand node) {
        boolean violation = this.violatesExecutableParameters(node.getExecutable(), node.getParameterList());

        if (violation) this.error(node);
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
