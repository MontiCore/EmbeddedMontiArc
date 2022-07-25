package de.monticore.lang.gdl.cocos.types;

import java.util.HashSet;
import java.util.Set;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameConstruct;
import de.monticore.lang.gdl._ast.ASTGameInference;
import de.monticore.lang.gdl._ast.ASTGameTuple;
import de.monticore.lang.gdl._ast.ASTGameTypeDef;
import de.monticore.lang.gdl._ast.ASTGameTypeMapDef;
import de.monticore.lang.gdl._cocos.GDLASTGameCoCo;
import de.se_rwth.commons.logging.Log;

public class TypesBoundOnce implements GDLASTGameCoCo {
    
    private Set<ASTGameConstruct> types = new HashSet<>();

    @Override
    public void check(ASTGame node) {
        for (ASTGameTuple tuple : node.getTuplesList()) {
            if (tuple.getElement(0) instanceof ASTGameTypeDef) {
                checkExisting(tuple.getElement(2));
            } else if (tuple.getElement(0) instanceof ASTGameInference && tuple.getElement(1) instanceof ASTGameTuple) {
                tuple = (ASTGameTuple) tuple.getElement(1);
                if (tuple.getElement(0) instanceof ASTGameTypeMapDef) {
                    checkExisting(tuple.getElement(2));
                }
            }
        }
    }

    private void checkExisting(ASTGameConstruct c) {
        if (types.contains(c)) {
            Log.error("Type is already defined! Types can only be defined once.", c.get_SourcePositionStart(), c.get_SourcePositionEnd());
        } else {
            types.add(c);
        }
    }

}
