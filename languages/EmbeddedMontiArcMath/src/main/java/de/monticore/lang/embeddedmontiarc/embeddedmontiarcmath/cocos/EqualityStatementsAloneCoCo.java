/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTBehaviorEmbedding;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTSpecification;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._cocos.EmbeddedMontiArcMathASTBehaviorEmbeddingCoCo;
import de.monticore.lang.math._ast.ASTStatement;
import de.se_rwth.commons.logging.Log;

public class EqualityStatementsAloneCoCo implements EmbeddedMontiArcMathASTBehaviorEmbeddingCoCo {
    @Override
    public void check(ASTBehaviorEmbedding node) {
        boolean isSpecification = false;
        boolean isAssignments = false;
        for (ASTStatement astStatement : node.getStatementList()) {
            if (astStatement instanceof ASTSpecification)
                isSpecification = true;
            else
                isAssignments = true;
        }
        if (isSpecification && isAssignments)
            Log.error("0xE4439862 Statements can only be of either specification kind or assignments");
    }
}
