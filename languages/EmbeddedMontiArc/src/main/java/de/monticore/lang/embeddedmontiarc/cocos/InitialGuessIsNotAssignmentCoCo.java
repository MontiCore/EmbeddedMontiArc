/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPortInitial;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTSubComponentCoCo;
import de.monticore.mcexpressions._ast.ASTAssignmentExpression;
import de.se_rwth.commons.logging.Log;

public class InitialGuessIsNotAssignmentCoCo implements EmbeddedMontiArcASTSubComponentCoCo {
    @Override
    public void check(ASTSubComponent node) {
        for (ASTPortInitial portInitial : node.getPortInitialList()) {
            if (portInitial.getExpression() instanceof ASTAssignmentExpression) {
                Log.error(String.format("0x079B7 Initial guess of has to be an assignment"),
                node.get_SourcePositionStart());
            }
        }

    }
}
