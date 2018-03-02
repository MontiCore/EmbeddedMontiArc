package de.monticore.reporting.testCocos.helper;

import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class ASTHelperTest {

    private final String MODEL_PATH = "src/test/resources/";

    @BeforeClass
    public static void before() {
        LogConfig.init();
        Log.enableFailQuick(false);
    }

    private void testParse_Positive(String file) throws IOException {
        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
        ASTEMAMCompilationUnit ast = null;
        ast = parser.parse(file).orElse(null);
        boolean res = ast != null;
        assertTrue("File \"" + file + "\" could not be parsed", res);
    }

    private void testParse_Negative(String file) {
        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
        ASTEMAMCompilationUnit ast = null;
        boolean parse = true;
        try {
            ast = parser.parse(file).orElse(null);
        } catch (IOException e) {
            parse = false;
        }
        boolean res = true;
        if (ast == null || !parse)
            res = false;
        assertFalse("File \"" + file + "\": expected parse error not occured", res);
    }

    private void testResolve_Positive(String file) throws IOException, CouldNotResolveException {
        ASTEmbeddedMontiArcNode astResolved = ASTHelper.getAstNode(file);
        boolean res = astResolved != null;
        assertTrue("File \"" + file + "\" could not be resolved.", res);
    }

    private void testResolve_Negative(String file) throws IOException {
        ASTEmbeddedMontiArcNode astResolved = null;
        boolean res = true;
        try {
            astResolved = ASTHelper.getAstNode(file);
        } catch (Exception e) {
            res = false;
        }
        res = astResolved != null || res;
        assertFalse("File \"" + file + "\": Expected resolve error did not occur.", res);
    }

    @Test
    public void testParseEMAM() throws IOException {
        String fileName = MODEL_PATH + "de/rwth/armin/modeling/autopilot/behavior/DriveToFirstPosition.emam";
        testParse_Positive(fileName);
    }

    @Test
    public void testParseEMAM_Negative() {
        String fileName = MODEL_PATH + "racingCar/racingCarModels/controller/BrakeComputation.emam";
        testParse_Negative(fileName);
    }

    @Test
    public void testParseEMA() throws IOException {
        String fileName = MODEL_PATH + "testing/BooleanConnector.ema";
        testParse_Positive(fileName);
    }

    @Test
    public void testParseEMA_Negative() {
        String fileName = MODEL_PATH + "testManuelSchrick/SubComponentConnector.ema";
        testParse_Negative(fileName);
    }

    @Test
    public void testResolveEMAM() throws IOException, CouldNotResolveException {
        String fileName = MODEL_PATH + "de/rwth/armin/modeling/autopilot/behavior/DriveToFirstPosition.emam";
        testResolve_Positive(fileName);
    }

    @Test
    public void testResolveEMAM_Negative() throws IOException {
        String fileName = MODEL_PATH + "racingCarModels/controller/Controller.emam";
        testResolve_Negative(fileName);
    }

    @Test
    public void testResolveEMA() throws IOException, CouldNotResolveException {
        String fileName = MODEL_PATH + "testManuelSchrick/BumperBot.ema";
        testResolve_Positive(fileName);
    }

    @Test
    public void testResolveEMA_Negative() throws IOException {
        String fileName = MODEL_PATH + "testing/ConnectorArray.ema";
        testResolve_Negative(fileName);
    }
}
