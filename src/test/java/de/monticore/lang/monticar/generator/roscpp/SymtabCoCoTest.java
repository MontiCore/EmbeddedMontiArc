package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.roscpp.cocos.EmbeddedMontiArcMathRosCppSymtabCoCos;
import de.monticore.lang.monticar.generator.roscpp.cocos.EmbeddedMontiArcMathSymtabCoCoChecker;
import de.monticore.lang.monticar.generator.roscpp.tagging.RosToEmamTagSchema;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class SymtabCoCoTest extends AbstractSymtabTest {

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
        testCoCo("tests.cocos.rosToRosComp", false);
    }

    @Test
    public void testNoRosToRos() {
        testCoCo("tests.cocos.noRosToRosComp", true);
    }

    @Test
    public void testTopicNameMismatch() {
        testCoCo("tests.cocos.topicNameMismatch", true);
    }

    @Test
    public void testTopicTypeMismatch() {
        testCoCo("tests.cocos.topicTypeMismatch", true);
    }

    @Test
    public void testPortTwoRosConnections() {
        testCoCo("tests.cocos.portTwoRosConnections", true);
    }


    //TODO: check findings for error msgs not just presence
    public void testCoCo(String componentInstanceName, boolean errorExpected) {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol component = taggingResolver.<ExpandedComponentInstanceSymbol>resolve(componentInstanceName, ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        EmbeddedMontiArcMathSymtabCoCoChecker symtabCoCoChecker = EmbeddedMontiArcMathRosCppSymtabCoCos.createSymtabChecker(taggingResolver);
        symtabCoCoChecker.checkAll(component);

        if (errorExpected) {
            assertTrue(Log.getErrorCount() > 0);
        } else {
            assertTrue(Log.getErrorCount() == 0);
        }

    }
}
