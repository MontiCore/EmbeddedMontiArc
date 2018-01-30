package de.monticore.lang.monticar.generator.roscpp.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;

public class InRosPortRosSender implements EmbeddedMontiArcASTComponentCoCo {
    @Override
    public void check(ASTComponent node) {
        InRosPortRosSenderChecker check = new InRosPortRosSenderChecker();
        check.check(node);
    }


    class InRosPortRosSenderChecker {
        void check(ASTComponent node) {
            //TODO: add check: connectors with ros ports as target only have ros ports as source(and topic matches)
        }

    }
}
