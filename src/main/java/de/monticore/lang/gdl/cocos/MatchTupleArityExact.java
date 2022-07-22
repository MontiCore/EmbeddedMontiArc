package de.monticore.lang.gdl.cocos;

import java.util.Map;

import de.monticore.lang.gdl._ast.ASTGameAdd;
import de.monticore.lang.gdl._ast.ASTGameConstruct;
import de.monticore.lang.gdl._ast.ASTGameDiv;
import de.monticore.lang.gdl._ast.ASTGameDoes;
import de.monticore.lang.gdl._ast.ASTGameEqual;
import de.monticore.lang.gdl._ast.ASTGameGoal;
import de.monticore.lang.gdl._ast.ASTGameGreater;
import de.monticore.lang.gdl._ast.ASTGameInit;
import de.monticore.lang.gdl._ast.ASTGameKeyword;
import de.monticore.lang.gdl._ast.ASTGameLegal;
import de.monticore.lang.gdl._ast.ASTGameLess;
import de.monticore.lang.gdl._ast.ASTGameMod;
import de.monticore.lang.gdl._ast.ASTGameMult;
import de.monticore.lang.gdl._ast.ASTGameNext;
import de.monticore.lang.gdl._ast.ASTGameNot;
import de.monticore.lang.gdl._ast.ASTGameNumber;
import de.monticore.lang.gdl._ast.ASTGameRole;
import de.monticore.lang.gdl._ast.ASTGameSees;
import de.monticore.lang.gdl._ast.ASTGameSub;
import de.monticore.lang.gdl._ast.ASTGameSucc;
import de.monticore.lang.gdl._ast.ASTGameTrue;
import de.monticore.lang.gdl._ast.ASTGameTuple;
import de.monticore.lang.gdl._ast.ASTGameTypeCombineDef;
import de.monticore.lang.gdl._ast.ASTGameTypeDef;
import de.monticore.lang.gdl._ast.ASTGameTypeMapDef;
import de.monticore.lang.gdl._cocos.GDLASTGameTupleCoCo;
import de.se_rwth.commons.logging.Log;

public class MatchTupleArityExact implements GDLASTGameTupleCoCo {

    // Exact count of elements after keyword
    private static final Map<Class<? extends ASTGameKeyword>, Integer> KEYWORD_ARITY_EXACT = Map.ofEntries(
        Map.entry(ASTGameRole.class, 1),
        Map.entry(ASTGameInit.class, 1),
        Map.entry(ASTGameNext.class, 1),
        Map.entry(ASTGameTrue.class, 1),
        Map.entry(ASTGameNot.class, 1),
        Map.entry(ASTGameNumber.class, 1),

        Map.entry(ASTGameTypeDef.class, 2),
        Map.entry(ASTGameSees.class, 2),
        Map.entry(ASTGameLegal.class, 2),
        Map.entry(ASTGameDoes.class, 2),
        Map.entry(ASTGameGoal.class, 2),
        Map.entry(ASTGameSucc.class, 2),
        Map.entry(ASTGameLess.class, 2),
        Map.entry(ASTGameGreater.class, 2),
        Map.entry(ASTGameEqual.class, 2),

        Map.entry(ASTGameAdd.class, 3),
        Map.entry(ASTGameSub.class, 3),
        Map.entry(ASTGameMult.class, 3),
        Map.entry(ASTGameDiv.class, 3),
        Map.entry(ASTGameMod.class, 3),
        Map.entry(ASTGameTypeMapDef.class, 3),
        Map.entry(ASTGameTypeCombineDef.class, 3)
    );

    @Override
    public void check(ASTGameTuple node) {
        ASTGameConstruct first = node.getElement(0);

        Integer match = KEYWORD_ARITY_EXACT.get(first.getClass());

        if (match != null) {
            if (node.getElementList().size() - 1 != match) {
                Log.error("Expected Tuple size was " + (match + 1) + ". Actual size is " + node.getElementList().size(),
                        node.get_SourcePositionStart(), node.get_SourcePositionEnd());
            }
        }
    }

}
