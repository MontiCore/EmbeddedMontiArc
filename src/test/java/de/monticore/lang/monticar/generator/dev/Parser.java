package de.monticore.lang.monticar.generator.dev;

import de.monticore.lang.monticar.generator.ParserTest;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import static org.junit.Assert.assertTrue;

public class Parser extends ParserTest{

    public static final boolean ENABLE_FAIL_QUICK = false;

    @Ignore
    @Test
    public void dev1() throws Exception {
        test("emam", "src/test/resources/fas/basicLibrary");
        if (Log.getErrorCount() > 0) {
            throw new Exception("Test Failed, found errors");
        }
    }

    private void test(String fileEnding, String path) throws IOException {
        ParserTest.ParseTest parserTest = new ParseTest("." + fileEnding);
        Files.walkFileTree(Paths.get(path), parserTest);

        if (!parserTest.getModelsInError().isEmpty()) {
            Log.debug("Models in error", "ParserTest");
            for (String model : parserTest.getModelsInError()) {
                Log.debug("  " + model, "ParserTest");
            }
        }
        Log.info("Count of tested models: " + parserTest.getTestCount(), "ParserTest");
        Log.info("Count of correctly parsed models: "
                + (parserTest.getTestCount() - parserTest.getModelsInError().size()), "ParserTest");

        assertTrue("There were models that could not be parsed", parserTest.getModelsInError()
                .isEmpty());
    }
}
