/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcCoCoChecker;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Test;

/**
 * Created by Sining on 2017/3/6.
 */
public class UniquePortsTest extends AbstractCoCoTest {
    @BeforeClass
    public static void setUp() {
        Log.enableFailQuick(false);
    }
    //@Ignore
    @Test
    public void testValid() {
        checkValid("", "testing.UniqueName");
    }

    //@Ignore
    @Test
    public void testInvalid() {
        checkInvalid(new EmbeddedMontiArcCoCoChecker().addCoCo(new SourceTargetNumberMatch()),
                getAstNode("", "testing.DuplicatePort"),
                new ExpectedErrorInfo());
    }
}
