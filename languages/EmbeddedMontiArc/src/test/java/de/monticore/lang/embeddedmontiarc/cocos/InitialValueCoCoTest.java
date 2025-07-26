/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcCoCoChecker;
import org.junit.Test;

public class InitialValueCoCoTest extends AbstractCoCoTest {

    private void checkModel(String model) {
        ASTEmbeddedMontiArcNode astNode = getAstNode("", model);

        EmbeddedMontiArcCoCoChecker checker = new EmbeddedMontiArcCoCoChecker();
        checker.addCoCo(new InitialGuessIsNotAssignmentCoCo());

        checker.checkAll(astNode);
    }

    @Test
    public void testValid() {
        checkModel("testing.PortInitialTest");
        checkModel("testing.PortInitialFromParameter");
    }
}
