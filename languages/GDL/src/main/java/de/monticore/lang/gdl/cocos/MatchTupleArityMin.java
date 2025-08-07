package de.monticore.lang.gdl.cocos;

import java.util.Map;
import de.monticore.lang.gdl._ast.ASTGameConstruct;
import de.monticore.lang.gdl._ast.ASTGameCount;
import de.monticore.lang.gdl._ast.ASTGameInference;
import de.monticore.lang.gdl._ast.ASTGameKeyword;
import de.monticore.lang.gdl._ast.ASTGameTuple;
import de.monticore.lang.gdl._cocos.GDLASTGameTupleCoCo;
import de.se_rwth.commons.logging.Log;

public class MatchTupleArityMin implements GDLASTGameTupleCoCo {

    // Minimum count of elements after keyword
    private static final Map<Class<? extends ASTGameKeyword>, Integer> KEYWORD_ARITY_MIN = Map.of(
        ASTGameInference.class, 1,
        ASTGameCount.class, 2
    );

    @Override
    public void check(ASTGameTuple node) {
        ASTGameConstruct first = node.getElement(0);

        Integer match = KEYWORD_ARITY_MIN.get(first.getClass());

        // Standard smallest tuple size = 1
        if (match == null) match = 0;

        if (node.getElementList().size() - 1 < match) {
            Log.error("Minimal Tuple size is >=" + (match + 1) + ". Actual size is " + node.getElementList().size(),
                    node.get_SourcePositionStart(), node.get_SourcePositionEnd());
        }
    }

}
