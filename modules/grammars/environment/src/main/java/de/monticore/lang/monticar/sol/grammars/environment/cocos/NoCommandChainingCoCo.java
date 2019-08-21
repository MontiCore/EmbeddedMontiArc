/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTCommandOrSplitCommand;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTRun;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTSplitCommand;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentASTRunCoCo;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.environment._visitor.EnvironmentVisitor;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.function.Predicate;

/**
 * This command checks whether RUN instructions have been chained instead of going for readability.
 */
public class NoCommandChainingCoCo implements EnvironmentCoCo, EnvironmentASTRunCoCo, EnvironmentVisitor {
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
    public void check(ASTRun node) {
        node.accept(getRealThis());
    }

    @Override
    public void visit(ASTCommandOrSplitCommand node) {
        node.getCommandOpt().ifPresent(command -> {
            if (this.violatesCommand(command)) Log.warn(this.getErrorMessage(), node.get_SourcePositionStart());
        });
    }

    @Override
    public void visit(ASTSplitCommand node) {
        boolean violation = this.violatesParameters(node.getParameterList());

        if (violation) Log.warn(this.getErrorMessage(), node.get_SourcePositionStart());
    }

    protected boolean violatesCommand(ASTStringLiteral command) {
        return command.getValue().contains("&&");
    }

    protected boolean violatesParameters(List<ASTStringLiteral> parameters) {
        return parameters.stream().map(ASTStringLiteral::getValue).anyMatch(Predicate.isEqual("&&"));
    }
}
