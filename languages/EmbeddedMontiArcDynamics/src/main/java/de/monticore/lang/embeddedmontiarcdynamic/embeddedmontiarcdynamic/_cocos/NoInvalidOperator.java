package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._cocos;

import de.monticore.commonexpressions._ast.ASTBooleanOrOpExpression;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTEventHandler;
import jline.internal.Log;

public class NoInvalidOperator implements EmbeddedMontiArcDynamicASTEventHandlerCoCo {

    @Override
    public void check(ASTEventHandler node) {
        if (node.getExpression() instanceof ASTBooleanOrOpExpression) {
            Log.warn("0xIO001 The ||-operator is currently not supported!");
        }
    }
}
