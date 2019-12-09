/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.grammar.cocos;

import de.monticore.ast.ASTNode;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class CommonCoCo<CoCoChecker> implements CoCo<CoCoChecker> {
    protected final String errorCode;
    protected final String schema;

    protected CommonCoCo(String errorCode, String schema) {
        this.errorCode = errorCode;
        this.schema = schema;
    }

    public String getErrorCode() {
        return this.errorCode;
    }

    protected String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format(String.format("%%s %s", this.schema), parameterList.toArray());
    }

    protected void warn(ASTNode node, Object... parameters) {
        Log.warn(this.getErrorMessage(parameters), node.get_SourcePositionStart());
    }

    protected void warn(Symbol symbol, Object... parameters) {
        symbol.getAstNode().ifPresent(node -> this.warn(node, parameters));
    }

    protected void error(ASTNode node, Object... parameters) {
        Log.error(this.getErrorMessage(parameters), node.get_SourcePositionStart());
    }

    protected void error(Symbol symbol, Object... parameters) {
        symbol.getAstNode().ifPresent(node -> this.error(node, parameters));
    }
}
