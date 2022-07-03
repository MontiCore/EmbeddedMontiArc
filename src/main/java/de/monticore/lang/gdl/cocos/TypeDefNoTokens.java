package de.monticore.lang.gdl.cocos;

import de.monticore.ast.ASTNode;
import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameKeyword;
import de.monticore.lang.gdl._ast.ASTGameToken;
import de.monticore.lang.gdl._ast.ASTGameTuple;
import de.monticore.lang.gdl._ast.ASTGameTypeDef;
import de.monticore.lang.gdl._cocos.GDLASTGameTokenCoCo;
import de.monticore.lang.gdl._symboltable.IGDLScope;
import de.se_rwth.commons.logging.Log;

public class TypeDefNoTokens implements GDLASTGameTokenCoCo {
    
    @Override
    public void check(ASTGameToken node) {
        ASTGameKeyword maybeTypeDef = getDefiningKeyword(node.getEnclosingScope());

        if (maybeTypeDef instanceof ASTGameTypeDef) {
            Log.error("No tokens allowed in type definition.", node.get_SourcePositionStart(), node.get_SourcePositionEnd());
        }
    }

    private ASTGameKeyword getDefiningKeyword(IGDLScope scope) {
        ASTNode node = scope.getAstNode();
        if (node instanceof ASTGame) {
            return null;
        }

        ASTGameKeyword parentKeyword = getDefiningKeyword(scope.getEnclosingScope());

        if (parentKeyword != null) {
            return parentKeyword;
        } else {
            ASTGameTuple tuple = (ASTGameTuple) node;
            return (ASTGameKeyword) tuple.getElement(0);
        }
    }

}
