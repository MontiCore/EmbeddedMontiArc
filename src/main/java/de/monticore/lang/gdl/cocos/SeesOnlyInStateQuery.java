package de.monticore.lang.gdl.cocos;

import de.monticore.lang.gdl._ast.ASTGameInit;
import de.monticore.lang.gdl._ast.ASTGameNext;
import de.monticore.lang.gdl._ast.ASTGameSees;
import de.monticore.lang.gdl._ast.ASTGameTrue;
import de.monticore.lang.gdl._ast.ASTGameTuple;
import de.monticore.lang.gdl._cocos.GDLASTGameSeesCoCo;
import de.se_rwth.commons.logging.Log;

public class SeesOnlyInStateQuery implements GDLASTGameSeesCoCo {

    @Override
    public void check(ASTGameSees node) {
        boolean error = false;
        ASTGameTuple ownTuple = (ASTGameTuple) node.getEnclosingScope().getAstNode();
        if (node.getEnclosingScope().getEnclosingScope().getAstNode() instanceof ASTGameTuple) {
            ASTGameTuple parentTuple = (ASTGameTuple) node.getEnclosingScope().getEnclosingScope().getAstNode();

            if (!((parentTuple.getElement(0) instanceof ASTGameTrue
                    || parentTuple.getElement(0) instanceof ASTGameNext
                    || parentTuple.getElement(0) instanceof ASTGameInit)
                    && parentTuple.getElement(1).equals(ownTuple))) {
                error = true;
            }
        } else {
            error = true;
        }

        if (error) {
            Log.error("Sees is only allowed in a state query.", node.get_SourcePositionStart(), node.get_SourcePositionEnd());
        }
    }
    
}
