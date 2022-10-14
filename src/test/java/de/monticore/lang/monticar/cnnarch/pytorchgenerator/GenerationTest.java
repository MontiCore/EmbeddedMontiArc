/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.pytorchgenerator;

import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.contrib.java.lang.system.ExpectedSystemExit;

import java.io.File;
import java.io.IOException;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import static junit.framework.TestCase.assertTrue;

public class GenerationTest {

    @Rule
    public final ExpectedSystemExit exit = ExpectedSystemExit.none();

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    public static void checkFilesAreEqual(Path generationPath, Path resultsPath, List<String> fileNames) {
        for (String fileName : fileNames){
            File genFile = new File(generationPath.toString() + "/" + fileName);
            File fileTarget = new File(resultsPath.toString() + "/" + fileName);
            Assert.assertTrue(areBothFilesEqual(genFile, fileTarget));
        }
    }

    public static boolean areBothFilesEqual(File file1, File file2) {
        if (!file1.exists()) {
            Assert.fail("file does not exist: " + file1.getAbsolutePath());
            return false;
        }
        if (!file2.exists()) {
            Assert.fail("file does not exist: " + file2.getAbsolutePath());
            return false;
        }
        List<String> lines1;
        List<String> lines2;
        try {
            lines1 = Files.readAllLines(file1.toPath());
            lines2 = Files.readAllLines(file2.toPath());
        } catch (IOException e) {
            e.printStackTrace();
            Assert.fail("IO error: " + e.getMessage());
            return false;
        }
        lines1 = discardEmptyLines(lines1);
        lines2 = discardEmptyLines(lines2);
        if (lines1.size() != lines2.size()) {
            Assert.fail(
                    "files have different number of lines: "
                            + file1.getAbsolutePath()
                            + " has " + lines1
                            + " lines and " + file2.getAbsolutePath() + " has " + lines2 + " lines"
            );
            return false;
        }
        int len = lines1.size();
        for (int i = 0; i < len; i++) {
            String l1 = lines1.get(i);
            String l2 = lines2.get(i);
            Assert.assertEquals("files differ in " + i + " line: "
                            + file1.getAbsolutePath()
                            + " has " + l1
                            + " and " + file2.getAbsolutePath() + " has " + l2,
                    l1,
                    l2
            );
        }
        return true;
    }

    private static List<String> discardEmptyLines(List<String> lines) {
        return lines.stream()
                .map(String::trim)
                .filter(l -> !l.isEmpty())
                .collect(Collectors.toList());
    }

    @Test
    public void testLeNetGeneration() throws IOException, TemplateException {
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

    @Test
    public void testCoraDglGeneration() {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "DGLNetwork", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2PyTorchCli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/coraDgl"),
                Arrays.asList(
                        "CNNNet_DGLNetwork.py"
                ));
    }

    @Test(expected = RuntimeException.class)
    public void testInvalidLayer() {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "LayerInvalid", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2PyTorchCli.main(args);
    }

    @Test(expected = RuntimeException.class)
    public void testInvalidArchitecture() {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "ArchitectureInvalid", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2PyTorchCli.main(args);
    }

}