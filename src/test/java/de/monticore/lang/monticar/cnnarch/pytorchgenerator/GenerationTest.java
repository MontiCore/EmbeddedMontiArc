/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.pytorchgenerator;

import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.contrib.java.lang.system.ExpectedSystemExit;

import java.io.IOException;

import java.nio.file.Paths;
import java.util.Arrays;

import static junit.framework.TestCase.assertTrue;

public class GenerationTest extends AbstractSymtabTest {

    @Rule
    public final ExpectedSystemExit exit = ExpectedSystemExit.none();

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testLeNet() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "LeNet", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2PyTorchCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNNet_LeNet.py",
                        "CNNPredictor_LeNet.h",
                        "execute_LeNet"));
    }

}