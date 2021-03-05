/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.se_rwth.commons.logging.Log;
import org.junit.Assert;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.Assert.assertTrue;

/**
 * Common methods for symboltable tests
 */
public class AbstractSymtabTest extends AbstractSymtab {
    public static void testFilesAreEqual(List<File> files, String restPath) {
        for (File f : files) {
            File fileTarget = new File("./src/test/resources/results/" + restPath + f.getName());
            if(f.getName().equals("Helper.h") || f.getName().equals("HelperA.h")){
                continue;
            }
            Log.debug("" + fileTarget.exists(), "Exists");
            Log.debug(f.getName() + " " + fileTarget.getName(), "Comparing");
            assertTrue(areBothFilesEqual(f, fileTarget));
        }
    }

    public static void checkFilesAreEqual(List<File> generatedFiles, Path generatedFilesBasePath, String resultsPath) {
        for (File f : generatedFiles) {
            Path relativePath = generatedFilesBasePath.relativize(f.toPath());
            File fileTarget = new File("./src/test/resources/results/" + resultsPath + relativePath.toString());
            assertTrue(areBothFilesEqual(f, fileTarget));
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
        lines1 = discardCopyrightNotice(lines1);
        lines2 = discardEmptyLines(lines2);
        lines2 = discardCopyrightNotice(lines2);

        String lines1AsString = String.join("\n", lines1);
        String lines2AsString = String.join("\n", lines2);

        if (lines1.size() != lines2.size()) {
            Assert.assertEquals(
                    "files have different number of lines: "
                            + file1.getAbsolutePath()
                            + " has " + lines1
                            + " lines and " + file2.getAbsolutePath() + " has " + lines2 + " lines",
                    lines1AsString,
                    lines2AsString
            );
            return false;
        }

        int len = lines1.size();
        for (int i = 0; i < len; i++) {
            String l1 = lines1.get(i).trim();
            String l2 = lines2.get(i).trim();
            if (!l1.equals(l2))
                Assert.assertEquals("files differ in line " + i + ": "
                                + file1.getAbsolutePath()
                                + " has " + l1
                                + " and " + file2.getAbsolutePath() + " has " + l2,
                        String.join("\n\n", l1, lines1AsString),
                        String.join("\n\n", l2, lines2AsString)
                );
        }
        return true;
    }

    private static List<String> discardCopyrightNotice(List<String> lines) {
        return lines
                .stream()
                .filter(s -> !s.contains("(c) https://github.com/MontiCore/monticore"))
                .collect(Collectors.toList());

    }

    private static List<String> discardEmptyLines(List<String> lines) {
        return lines.stream()
                .map(String::trim)
                .filter(l -> !l.isEmpty())
                .collect(Collectors.toList());
    }
}
