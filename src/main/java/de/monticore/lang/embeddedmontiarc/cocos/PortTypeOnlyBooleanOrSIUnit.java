/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPort;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTPortCoCo;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.ranges._ast.ASTRanges;
import de.monticore.types.types._ast.ASTPrimitiveType;
import de.monticore.types.types._ast.ASTType;
import de.se_rwth.commons.logging.Log;

/**
 * Created by dennisqiao on 2/9/17.
 */
public class PortTypeOnlyBooleanOrSIUnit implements EmbeddedMontiArcASTPortCoCo{

    @Override
    public void check(ASTPort node) {
        ASTType type = node.getType();
        if (!(type instanceof ASTRange) && !(type instanceof ASTRanges) && (type instanceof ASTPrimitiveType && !type.toString().equals("Boolean"))) {
            Log.error(String.format("0xAE753 Port type can only be Boolean or a SI_Unit"), node.get_SourcePositionStart());
        }
    }
}
