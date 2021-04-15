/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos;

//import de.monticore.lang.embeddedmontiarc.cocos.SourceTargetNumberMatch;
//import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcCoCoChecker;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._cocos.EmbeddedMontiArcMathCoCoChecker;
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
    //@Ignore
    @Test
    public void testValid() {
        checkValid("", "detection.EigenSolver");
    }

    @Ignore
    @Test
    public void testInvalid() {
        checkInvalid(new EmbeddedMontiArcMathCoCoChecker().addCoCo(new AtomicComponentCoCo()),
                getAstNode("", "detection.EigenSolver"),
                new ExpectedErrorInfo());
    }
}
