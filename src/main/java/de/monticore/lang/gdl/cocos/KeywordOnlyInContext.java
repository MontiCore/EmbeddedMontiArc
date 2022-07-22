package de.monticore.lang.gdl.cocos;

import java.util.Set;
import java.util.function.Predicate;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameAdd;
import de.monticore.lang.gdl._ast.ASTGameConstruct;
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
import de.monticore.lang.gdl._ast.ASTGameNoop;
import de.monticore.lang.gdl._ast.ASTGameNot;
import de.monticore.lang.gdl._ast.ASTGameNumber;
import de.monticore.lang.gdl._ast.ASTGameRole;
import de.monticore.lang.gdl._ast.ASTGameSees;
import de.monticore.lang.gdl._ast.ASTGameSub;
import de.monticore.lang.gdl._ast.ASTGameSucc;
import de.monticore.lang.gdl._ast.ASTGameTerminal;
import de.monticore.lang.gdl._ast.ASTGameTrue;
import de.monticore.lang.gdl._ast.ASTGameTuple;
import de.monticore.lang.gdl._ast.ASTGameTypeCombineDef;
import de.monticore.lang.gdl._ast.ASTGameTypeDef;
import de.monticore.lang.gdl._ast.ASTGameTypeMapDef;
import de.monticore.lang.gdl._cocos.GDLASTGameKeywordCoCo;
import de.se_rwth.commons.logging.Log;

public class KeywordOnlyInContext implements GDLASTGameKeywordCoCo {

    private static final Set<Context> CONTEXTS = Set.of(
        new Context(
            "root",
            t -> (!(t instanceof ASTGameTerminal)
                    && t.getEnclosingScope().getEnclosingScope().getAstNode() instanceof ASTGame),
            Set.of(
                ASTGameInference.class,
                ASTGameInit.class,
                ASTGameRole.class,
                
                ASTGameTypeDef.class,
                ASTGameTypeCombineDef.class
            )
        ),
        new Context(
            "state",
            createDirectContextPredicate(
                Set.of(
                    ASTGameInit.class,
                    ASTGameNext.class,
                    ASTGameTrue.class
                )
            ),
            Set.of(
                ASTGameSees.class
            )
        ),
        new Context(
            "rule body",
            t -> {
                if (t.getEnclosingScope().getEnclosingScope().getAstNode() instanceof ASTGame) return false;
                ASTGameTuple parent = (ASTGameTuple) t.getEnclosingScope().getEnclosingScope().getAstNode();
                ASTGameConstruct first = parent.getElement(0);

                if (first instanceof ASTGameNot) {
                    return true;
                }

                if (first instanceof ASTGameInference || first instanceof ASTGameCount) {
                    ASTGameConstruct second = parent.getElement(1);
                    ASTGameTuple ownTuple = (ASTGameTuple) t.getEnclosingScope().getAstNode();

                    if (second != ownTuple) {
                        return true;
                    }
                }

                return false;
            },
            Set.of(
                ASTGameNot.class,
                ASTGameDistinct.class,

                ASTGameTrue.class,
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
                ASTGameNumber.class,

                ASTGameTerminal.class,
                ASTGameRole.class,
                ASTGameLegal.class,
                ASTGameDoes.class,
                ASTGameGoal.class,

                ASTGameTypeDef.class,
                ASTGameTypeMapDef.class
            )
        ),
        new Context(
            "inferred rule",
            t -> {
                if (t.getEnclosingScope().getEnclosingScope().getAstNode() instanceof ASTGame) {
                    ASTGameTuple ownTuple = (ASTGameTuple) t.getEnclosingScope().getAstNode();
                    return (t instanceof ASTGameTerminal)
                            && (ownTuple.getElement(0) instanceof ASTGameInference)
                            && ownTuple.getElement(1) == t;
                }

                ASTGameTuple parent = (ASTGameTuple) t.getEnclosingScope().getEnclosingScope().getAstNode();
                ASTGameConstruct first = parent.getElement(0);

                if (first instanceof ASTGameInference) {
                    ASTGameConstruct second = parent.getElement(1);

                    ASTGameTuple ownTuple = (ASTGameTuple) t.getEnclosingScope().getAstNode();
                    if (second == ownTuple) {
                        return true;
                    }
                }

                return false;
            },
            Set.of(
                ASTGameTerminal.class,
                ASTGameNext.class,
                ASTGameLegal.class,
                ASTGameGoal.class,
                ASTGameTypeMapDef.class
            )
        )
    );

    private static final Context DATA_TUPLE_CONTEXT = new Context(
        "data tuple",
        t -> false,
        Set.of(
            ASTGameNoop.class
        )
    );

    @Override
    public void check(ASTGameKeyword node) {
        Context context = DATA_TUPLE_CONTEXT;

        for (Context c : CONTEXTS) {
            if (c.isInContext(node)) {
                context = c;
                break;
            } 
        }

        if (!context.getAllowedKeywords().contains(node.getClass())) {
            Log.error("The keyword is not allowed in the " + context.getName() + " context",
                    node.get_SourcePositionStart(), node.get_SourcePositionEnd());
        }
    }

    private static Predicate<ASTGameKeyword> createDirectContextPredicate(final Set<Class<? extends ASTGameKeyword>> indicators) {
        return t -> {
            if (t.getEnclosingScope().getEnclosingScope().getAstNode() instanceof ASTGame) return false;

            ASTGameTuple parent = (ASTGameTuple) t.getEnclosingScope().getEnclosingScope().getAstNode();
            return indicators.contains(parent.getElement(0).getClass());
        };
    }

    private static class Context {

        private final String name;
        private final Predicate<ASTGameKeyword> isInContext;
        private final Set<Class<? extends ASTGameKeyword>> allowedKeywords;


        private Context(String name, Predicate<ASTGameKeyword> isInContext, Set<Class<? extends ASTGameKeyword>> allowedKeywords) {
            this.name = name;
            this.isInContext = isInContext;
            this.allowedKeywords = allowedKeywords;
        }

        public Set<Class<? extends ASTGameKeyword>> getAllowedKeywords() {
            return allowedKeywords;
        }

        public boolean isInContext(ASTGameKeyword keyword) {
            return this.isInContext.test(keyword);
        }

        public String getName() {
            return name;
        }

    }
    
}
