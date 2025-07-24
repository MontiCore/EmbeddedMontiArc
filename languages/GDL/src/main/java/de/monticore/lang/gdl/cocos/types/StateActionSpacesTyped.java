package de.monticore.lang.gdl.cocos.types;

import java.util.function.Function;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameConstruct;
import de.monticore.lang.gdl._ast.ASTGameInference;
import de.monticore.lang.gdl._ast.ASTGameInit;
import de.monticore.lang.gdl._ast.ASTGameLegal;
import de.monticore.lang.gdl._ast.ASTGameNext;
import de.monticore.lang.gdl._ast.ASTGameSees;
import de.monticore.lang.gdl._ast.ASTGameToken;
import de.monticore.lang.gdl._ast.ASTGameTuple;
import de.monticore.lang.gdl._ast.ASTGameValue;
import de.monticore.lang.gdl._cocos.GDLASTGameCoCo;
import de.se_rwth.commons.logging.Log;

public class StateActionSpacesTyped implements GDLASTGameCoCo {
    
    private static final Function<ASTGameConstruct, ASTGameConstruct> HIDDEN_CONSTRUCT_MAPPER = c -> {
        if (c instanceof ASTGameTuple) {
            ASTGameTuple tuple = (ASTGameTuple) c;
            if (tuple.getElement(0) instanceof ASTGameSees) {
                // skip invisible states
                if (tuple.getElement(1) instanceof ASTGameValue && ((ASTGameValue) tuple.getElement(1)).getValue().equals("none")) {
                    return null;
                }
                return tuple.getElement(2);
            }
            return c;
        } else {
            return c;
        }
    };

    @Override
    public void check(ASTGame node) {
        // legal
        node.getTuplesList().stream()
            .filter(t -> t.getElement(0) instanceof ASTGameInference && t.getElement(1) instanceof ASTGameTuple)
            .map(t -> (ASTGameTuple) t.getElement(1))
            .filter(t -> t.getElement(0) instanceof ASTGameLegal)
            // skip random
            .filter(t -> !(t.getElement(1) instanceof ASTGameValue && ((ASTGameValue) t.getElement(1)).getValue().equals("random")))
            .map(t -> t.getElement(2))
            .forEach(this::checkTyped);

        // init
        node.getTuplesList().stream()
            .filter(t -> t.getElement(0) instanceof ASTGameInit)
            .map(t -> t.getElement(1))
            .map(HIDDEN_CONSTRUCT_MAPPER)
            .filter(t -> t != null)
            .forEach(this::checkTyped);
            
        // next
        node.getTuplesList().stream()
            .filter(t -> t.getElement(0) instanceof ASTGameInference && t.getElement(1) instanceof ASTGameNext)
            .map(t -> (ASTGameTuple) t.getElement(1))
            .map(t -> t.getElement(1))
            .map(HIDDEN_CONSTRUCT_MAPPER)
            .filter(t -> t != null)
            .forEach(this::checkTyped);
    }

    private void checkTyped(ASTGameConstruct construct) {
        if (construct instanceof ASTGameToken) {
            checkTyped((ASTGameToken) construct);
        } else if (construct instanceof ASTGameTuple) {
            checkTyped((ASTGameTuple) construct);
        }
    }

    private void checkTyped(ASTGameToken construct) {
        if (!construct.isPresentType()) {
            Log.error("Token was expected to be typed", construct.get_SourcePositionStart(), construct.get_SourcePositionEnd());
        }
    }

    private void checkTyped(ASTGameTuple construct) {
        if (!construct.isPresentType()) {
            construct.getElementList().forEach(this::checkTyped);
        }
    }

}
