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

public class DataPathConfigParserTest extends AbstractSymtabTest {

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testDataPathConfigParserValidComponent() {
        DataPathConfigParser parser = new DataPathConfigParser("src/test/resources/architectures/data_paths.txt");

        String data_path = parser.getDataPath("ComponentName");
        assertTrue("Wrong data path returned", data_path.equals("/path/to/training/data"));
    }

    @Test
    public void testDataPathConfigParserInvalidComponent() {
        DataPathConfigParser parser = new DataPathConfigParser("src/test/resources/architectures/data_paths.txt");

        String data_path = parser.getDataPath("NotExistingComponent");
        assertTrue("For not listed components, null should be returned", data_path == null);
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void testDataPathConfigParserInvalidPath() {
        DataPathConfigParser parser = new DataPathConfigParser("invalid/path/data_paths.txt");

        assertTrue(Log.getFindings().size() == 1);
    }
}
