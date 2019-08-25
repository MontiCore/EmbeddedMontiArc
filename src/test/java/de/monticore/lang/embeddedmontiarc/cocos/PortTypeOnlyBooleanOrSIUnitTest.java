/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcCoCoChecker;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Test;

/**
 * Created by Sining on 2017/2/9.
 */
public class PortTypeOnlyBooleanOrSIUnitTest extends AbstractCoCoTest {

    @BeforeClass
    public static void setUp() {
        Log.enableFailQuick(false);
    }

    //TODO remove ignore later
    //@Ignore
    @Test
    public void testValid() {
        checkValid("", "testing.BooleanPortType");
    }

    //@Ignore
    @Test
    public void testInvalid() {
        checkInvalid(new EmbeddedMontiArcCoCoChecker().addCoCo(new PortTypeOnlyBooleanOrSIUnit()),
                getAstNode("", "testing.IntegerPortType"),
                new ExpectedErrorInfo());
    }
}
