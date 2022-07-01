package de.monticore.lang.gdl.cocos;

import java.util.Set;

import de.monticore.lang.gdl._ast.ASTGameAdd;
import de.monticore.lang.gdl._ast.ASTGameCount;
import de.monticore.lang.gdl._ast.ASTGameDistinct;
import de.monticore.lang.gdl._ast.ASTGameDiv;
import de.monticore.lang.gdl._ast.ASTGameDoes;
import de.monticore.lang.gdl._ast.ASTGameEqual;
import de.monticore.lang.gdl._ast.ASTGameGoal;
import de.monticore.lang.gdl._ast.ASTGameGreater;
import de.monticore.lang.gdl._ast.ASTGameInference;
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
import de.monticore.lang.gdl._cocos.GDLASTGameKeywordCoCo;
import de.se_rwth.commons.logging.Log;

public class KeywordsOnFirstPositionOnly implements GDLASTGameKeywordCoCo {

    private static final Set<Class<? extends ASTGameKeyword>> FILTER = Set.of(
        ASTGameInference.class,
        ASTGameRole.class,
        ASTGameInit.class,
        ASTGameNext.class,
        ASTGameTrue.class,
        ASTGameSees.class,
        ASTGameLegal.class,
        ASTGameDoes.class,
        ASTGameNot.class,
        ASTGameDistinct.class,
        ASTGameGoal.class,
        ASTGameCount.class,
        ASTGameAdd.class,
        ASTGameSub.class,
        ASTGameMult.class,
        ASTGameDiv.class,
        ASTGameMod.class,
        ASTGameSucc.class,
        ASTGameLess.class,
        ASTGameGreater.class,
        ASTGameEqual.class,
        ASTGameNumber.class
    );

    @Override
    public void check(ASTGameKeyword keyword) {
        if (!FILTER.contains(keyword.getClass())) return;

        ASTGameTuple ownTuple = (ASTGameTuple) keyword.getEnclosingScope().getAstNode();

        if (!ownTuple.getElement(0).equals(keyword)) {
            Log.error("The keyword was expected at the first index of a tuple", keyword.get_SourcePositionStart(), keyword.get_SourcePositionEnd());
        }
    }
    
}
