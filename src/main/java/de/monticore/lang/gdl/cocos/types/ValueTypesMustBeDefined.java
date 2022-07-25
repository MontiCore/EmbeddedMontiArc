package de.monticore.lang.gdl.cocos.types;

import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameInference;
import de.monticore.lang.gdl._ast.ASTGameTuple;
import de.monticore.lang.gdl._ast.ASTGameTypeCombineDef;
import de.monticore.lang.gdl._ast.ASTGameTypeDef;
import de.monticore.lang.gdl._ast.ASTGameTypeMapDef;
import de.monticore.lang.gdl._ast.ASTGameValue;
import de.monticore.lang.gdl._ast.ASTGameValueType;
import de.monticore.lang.gdl._cocos.GDLASTGameValueTypeCoCo;
import de.se_rwth.commons.logging.Log;

public class ValueTypesMustBeDefined implements GDLASTGameValueTypeCoCo {

    private ASTGame game = null;
    private Set<String> allTypes = null;

    public void setASTGame(ASTGame game) {
        this.game = game;
        this.allTypes = createAllTypes();
    }

    @Override
    public void check(ASTGameValueType node) {
        if (game == null) {
            return;
        }

        if (!allTypes.contains(node.getType())) {
            Log.error("Type is not defined!", node.get_SourcePositionStart(), node.get_SourcePositionEnd());
        }
    }

    private Set<String> createAllTypes() {
        Set<String> staticTypes = game.getTuplesList().stream()
            .filter(t -> t.getElement(0) instanceof ASTGameTypeDef || t.getElement(0) instanceof ASTGameTypeCombineDef)
            .map(t -> t.getElement(1))
            .map(v -> ((ASTGameValue) v).getValue())
            .collect(Collectors.toSet());

        Set<String> mappedTypes = game.getTuplesList().stream()
            .filter(t -> t.getElement(0) instanceof ASTGameInference && t.getElement(1) instanceof ASTGameTuple)
            .map(t -> (ASTGameTuple) t.getElement(1))
            .filter(t -> t.getElement(0) instanceof ASTGameTypeMapDef) 
            .map(t -> t.getElement(1))
            .map(v -> ((ASTGameValue) v).getValue())
            .collect(Collectors.toSet());

        staticTypes =  new HashSet<>(staticTypes);
        staticTypes.addAll(mappedTypes);

        return staticTypes;
    }

}
