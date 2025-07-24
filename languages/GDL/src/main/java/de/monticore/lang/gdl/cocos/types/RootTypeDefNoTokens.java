package de.monticore.lang.gdl.cocos.types;

import de.monticore.ast.ASTNode;
import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameToken;
import de.monticore.lang.gdl._ast.ASTGameTuple;
import de.monticore.lang.gdl._ast.ASTGameTypeDef;
import de.monticore.lang.gdl._cocos.GDLASTGameTokenCoCo;
import de.se_rwth.commons.logging.Log;

public class RootTypeDefNoTokens implements GDLASTGameTokenCoCo {
    
    @Override
    public void check(ASTGameToken node) {
        ASTGameTuple contextNode = (ASTGameTuple) contextNode(node);

        if (contextNode.getElement(0) instanceof ASTGameTypeDef) {
            Log.error("Tokens are not allowed in type definitions.",
                    node.get_SourcePositionStart(), node.get_SourcePositionEnd());
        }
    }

    private ASTNode contextNode(ASTNode node) {
        ASTNode parent = node.getEnclosingScope().getAstNode();
        if (parent instanceof ASTGame) {
            return node;
        } else {
            return contextNode(node.getEnclosingScope().getAstNode());
        }
    }

}
