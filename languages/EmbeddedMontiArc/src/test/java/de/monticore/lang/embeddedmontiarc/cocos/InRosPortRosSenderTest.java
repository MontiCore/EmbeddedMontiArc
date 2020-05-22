/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.se_rwth.commons.logging.Log;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class InRosPortRosSenderTest extends AbstractTaggingCoCoTest{

    @BeforeClass
    public static void init() {
        Log.enableFailQuick(false);
    }

    @Before
    public void setUp() {
        Log.getFindings().clear();
    }

    @Test
    public void testValidRosToRos() {
        testCoCosOnComponent("middleware.ros.cocos.RosToRosComp");
    }

    @Test
    public void testNoRosToRos() {
        testCoCosOnComponent("middleware.ros.cocos.NoRosToRosComp", "0x3830a");
    }

    @Test
    public void testTopicNameMismatch() {
        testCoCosOnComponent("middleware.ros.cocos.TopicNameMismatch", "0x23a0d");
    }

    @Test
    public void testTopicTypeMismatch() {
        testCoCosOnComponent("middleware.ros.cocos.TopicTypeMismatch", "0x31f6e");
    }

    @AfterClass
    public static void finish(){
        Log.getFindings().clear();
    }
}
