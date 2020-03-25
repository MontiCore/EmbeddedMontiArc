/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar;

import de.monticore.lang.monticar.test._ast.ASTA;
import de.monticore.lang.monticar.test._ast.ASTC;
import de.monticore.lang.monticar.test._ast.ASTD;
import de.monticore.lang.monticar.test._parser.TestParser;
import de.se_rwth.commons.logging.Log;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.assertNotNull;

/**
 */
public class TestParserTest {

    static boolean failQuick;

    @BeforeClass
    public static void startUp() {
        failQuick = Log.isFailQuickEnabled();
        Log.enableFailQuick(false);
    }

    @AfterClass
    public static void tearDown() {
        Log.enableFailQuick(failQuick);
    }

    @Before
    public void clear() {
        Log.getFindings().clear();
    }

    @Test
    public void testA() throws IOException {
        TestParser parser = new TestParser();
        ASTA ast = parser.parse_StringA("A.*;").orElse(null);
        assertNotNull(ast);
    }

    @Test
    public void testC() throws IOException {
        TestParser parser = new TestParser();
        ASTC ast = parser.parse_StringC("C;").orElse(null);
        assertNotNull(ast);
    }

    @Test
    public void testD() throws IOException {
        TestParser parser = new TestParser();
        ASTD ast = parser.parse_StringD("D;").orElse(null);
        assertNotNull(ast);
    }


    /*@Test
    public void testB() throws IOException {
        TestParser parser = new TestParser();
        ASTB ast = parser.parseString_B("B.*;").orElse(null);
        assertNotNull(ast);
    }*/

}

