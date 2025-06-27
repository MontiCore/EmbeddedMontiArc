/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedVariables;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class WeightsPathConfigParserTest extends AbstractSymtabTest {

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testWeightsPathConfigParserValidComponent() {
        WeightsPathConfigParser parser = new WeightsPathConfigParser("src/test/resources/architectures/weights_paths.txt");

        String weights_path = parser.getWeightsPath("ComponentName");
        assertTrue("Wrong weights path returned", weights_path.equals("/path/to/training/weights"));
    }

    @Test
    public void testWeightsPathConfigParserInvalidPath() {
        WeightsPathConfigParser parser = new WeightsPathConfigParser("invalid/path/weights_paths.txt");

        assertTrue(Log.getFindings().size() == 1);
    }
}
