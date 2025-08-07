package de.monticore.lang.gdl.cocos.types;

import de.monticore.ast.ASTNode;
import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameInference;
import de.monticore.lang.gdl._ast.ASTGameToken;
import de.monticore.lang.gdl._ast.ASTGameTuple;
import de.monticore.lang.gdl._ast.ASTGameTypeMapDef;
import de.monticore.lang.gdl._cocos.GDLASTGameTokenCoCo;
import de.se_rwth.commons.logging.Log;

public class TypeMapDefNoTokens implements GDLASTGameTokenCoCo {

    @Override
    public void check(ASTGameToken node) {
        boolean flag = flagNode(node, false);

        if (flag) {
            Log.error("Tokens are not allowed at this position.",
                    node.get_SourcePositionStart(), node.get_SourcePositionEnd());
        }
    }

    private boolean flagNode(ASTNode node, boolean sus) {
        ASTNode parent = node.getEnclosingScope().getAstNode();

        if (sus && parent instanceof ASTGameTuple
            && ((ASTGameTuple) parent).getElement(0) instanceof ASTGameInference
            && ((ASTGameTuple) parent).getElement(1) == node) {
            return true;
        }

        if (parent instanceof ASTGameTuple
            && ((ASTGameTuple) parent).getElement(0) instanceof ASTGameTypeMapDef
            && (
                ((ASTGameTuple) parent).getElement(1) == node
                || ((ASTGameTuple) parent).getElement(2) == node
            )
        ) {
            return flagNode(node.getEnclosingScope().getAstNode(), true);
        }

        if (parent instanceof ASTGame) {
            return false;
        } else {
            return flagNode(node.getEnclosingScope().getAstNode(), false);
        }
    }

}