package de.monticore.lang.gdl.cocos;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameInit;
import de.monticore.lang.gdl._cocos.GDLASTGameInitCoCo;
import de.se_rwth.commons.logging.Log;

public class InitOnlyOnRoot implements GDLASTGameInitCoCo {

    @Override
    public void check(ASTGameInit node) {
        if (!(node.getEnclosingScope().getEnclosingScope().getAstNode() instanceof ASTGame)) {
            Log.error("Init is only allowed in root tuples", node.get_SourcePositionStart(), node.get_SourcePositionEnd());
        }
    }
    
}
