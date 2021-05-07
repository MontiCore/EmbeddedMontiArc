/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl.integration;

import de.monticore.lang.monticar.emadl.AbstractSymtabTest;
import de.monticore.lang.monticar.emadl.generator.Backend;
import de.monticore.lang.monticar.emadl.generator.EMADLGenerator;
import de.monticore.lang.monticar.emadl.generator.EMADLGeneratorCli;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.apache.commons.io.FileUtils;
import org.junit.*;

import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertFalse;

public abstract class IntegrationTest extends AbstractSymtabTest {

    private String backend;
    private String trainingHash;

    @BeforeClass
    public static void setupClass() throws IOException {
        if (new File("model").exists()) {
            FileUtils.deleteDirectory(new File("model"));
        }
    }

    @AfterClass
    public static void tearDown() throws IOException {
        if (new File("model").exists()) {
            FileUtils.deleteDirectory(new File("model"));
        }
    }

    public IntegrationTest(String backend, String trainingHash) {
        this.backend = backend;
        this.trainingHash = trainingHash;
    }

    private Path netTrainingHashFile = Paths.get("./target/generated-sources-emadl/simpleCifar10/CifarNetwork.training_hash");

    private void createHashFile() {
        try {
            netTrainingHashFile.toFile().getParentFile().mkdirs();
            List<String> lines = Arrays.asList(this.trainingHash);
            Files.write(netTrainingHashFile, lines, Charset.forName("UTF-8"));
        }
        catch(Exception e) {
            assertFalse("Hash file could not be created", true);
        }
    }

    private void deleteHashFile() {
        try {
            Files.delete(netTrainingHashFile);
        }
        catch(Exception e) {
            assertFalse("Could not delete hash file", true);
        }
    }

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testDontRetrain1() {
        // The training hash is stored during the first training, so the second one is skipped
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "simpleCifar10.Cifar10Classifier", "-b", this.backend};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
        
        Log.getFindings().clear();
        EMADLGeneratorCli.main(args);
        checkFindingsCount(1);
        assertTrue(Log.getFindings().get(0).getMsg().contains("skipped"));

        deleteHashFile();
    }

    @Test
    @Ignore
    public void testDontRetrain2() {
        // The training hash is written manually, so even the first training should be skipped
        Log.getFindings().clear();
        createHashFile();

        String[] args = {"-m", "src/test/resources/models/", "-r", "simpleCifar10.Cifar10Classifier", "-b", this.backend};
        EMADLGeneratorCli.main(args);
        //assertTrue(Log.getFindings().size() == 1);
        //assertTrue(Log.getFindings().get(0).getMsg().contains("skipped"));

        deleteHashFile();
    }

    @Test
    public void testDontRetrain3() {
        // Multiple instances of the first NN are used. Only the first one should cause a training
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "instanceTestCifar.MainC", "-b", this.backend};
        EMADLGeneratorCli.main(args);
        checkFindingsCount(1);
        assertTrue(Log.getFindings().get(0).getMsg().contains("skipped"));
        deleteInstanceTestCifarHashFile();
    }


    private void deleteInstanceTestCifarHashFile() {
        final Path instanceTestCifarHasFile
                = Paths.get("./target/generated-sources-emadl/instanceTestCifar/CifarNetwork.training_hash");
        try {
            Files.delete(instanceTestCifarHasFile);
        }
        catch(Exception e) {
            assertFalse("Could not delete hash file", true);
        }
    }

    @Test
    public void testForceRetrain() {
        // The training hash is written manually, but training is forced
        Log.getFindings().clear();
        createHashFile();

        String[] args = {"-m", "src/test/resources/models/", "-r", "simpleCifar10.Cifar10Classifier", "-b", this.backend, "-f", "y"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        deleteHashFile();
    }
}
