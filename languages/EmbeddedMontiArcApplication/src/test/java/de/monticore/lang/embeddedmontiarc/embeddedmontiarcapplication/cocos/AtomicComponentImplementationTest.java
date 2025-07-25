/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication.cocos;

import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

/**
 */
public class AtomicComponentImplementationTest extends AbstractCoCoTest {
    @BeforeClass
    public static void setUp() {
        Log.enableFailQuick(false);
    }

    @Ignore
    @Test
    public void testValid() {
        checkValid("", "detection.EigenSolver");
    }

    @Ignore
    @Test
    public void testInvalid() {
        /*checkInvalid(new EmbeddedMontiArcApplicationCoCoChecker().addCoCo(new AtomicComponentCoCo()),
                getAstNode("", "detection.EigenSolver"),
                new ExpectedErrorInfo());
    */}
}
