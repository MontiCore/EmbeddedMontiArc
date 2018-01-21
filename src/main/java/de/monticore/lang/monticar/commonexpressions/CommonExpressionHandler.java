package de.monticore.lang.monticar.commonexpressions;

import de.monticore.lang.monticar.commonexpressions._ast.ASTCommonExpression;

/**
 * @author Sascha Schneiders
 */
public interface CommonExpressionHandler {
    void handleExpression(ASTCommonExpression astCommonExpression);
}
