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
import java.util.function.Predicate;

/**
 * This command checks whether RUN instructions have been chained instead of going for readability.
 */
public class NoCommandChainingCoCo extends CommonEnvironmentCoCo implements EnvironmentASTRunCoCo, EnvironmentVisitor {
    public NoCommandChainingCoCo() {
        super("ENV0003", "RUN should have at most one command.");
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
            if (this.violatesCommand(command)) this.warn(node);
        });
    }

    @Override
    public void visit(ASTSplitCommand node) {
        boolean violation = this.violatesParameters(node.getParameterList());

        if (violation) this.warn(node);
    }

    protected boolean violatesCommand(ASTStringLiteral command) {
        return command.getValue().contains("&&");
    }

    protected boolean violatesParameters(List<ASTStringLiteral> parameters) {
        return parameters.stream().map(ASTStringLiteral::getValue).anyMatch(Predicate.isEqual("&&"));
    }
}
