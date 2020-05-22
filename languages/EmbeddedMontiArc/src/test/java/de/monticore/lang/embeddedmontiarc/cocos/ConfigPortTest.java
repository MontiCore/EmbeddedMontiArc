/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Test;

public class ConfigPortTest extends AbstractCoCoTest {

    @BeforeClass
    public static void setUp() {
        Log.enableFailQuick(false);
    }

    @Test
    public void testConfigPortMustNotBeConnected(){
        checkValid("","testing.AdaptableParameterInstance");
    }

    @Test
    public void testConfigPortOnlyIncomingIsConfig(){
        checkValid("","testing.ConfigPort");
    }

    @Test
    public void testConfigPortOutgoingIsConfig(){


        ASTEmbeddedMontiArcNode astNode = getAstNode("", "testing.ConfigPort");

        //set output port to config. Mistake can only happen when using Tagging as parser will catch it
        EMAComponentSymbol comp = (EMAComponentSymbol) astNode.getSymbol().get();
        EMAPortSymbol outPort = comp.getOutgoingPort("out1").get();
        outPort.setConfig(true);

        ExpectedErrorInfo expectedErrors = new ExpectedErrorInfo(1,"x7FF02");
        checkInvalid(EmbeddedMontiArcCoCos.createChecker(), astNode,expectedErrors);


    }
}
