/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.tensorflowgenerator;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchCompilationUnitSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import org.junit.Assert;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

/**
 * Common methods for symboltable tests
 */
public class AbstractSymtabTest {

    private static final String MODEL_PATH = "src/test/resources/";

    protected static Scope createSymTab(String... modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();

        fam.addModelingLanguage(new CNNArchLanguage());

        final ModelPath mp = new ModelPath();
        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }

        return new GlobalScope(mp, fam);
    }

    protected static CNNArchCompilationUnitSymbol getCompilationUnitSymbol(String modelPath, String model) {
        Scope symTab = createSymTab(MODEL_PATH + modelPath);
        CNNArchCompilationUnitSymbol comp = symTab.<CNNArchCompilationUnitSymbol> resolve(
                model, CNNArchCompilationUnitSymbol.KIND).orElse(null);
        assertNotNull("Could not resolve model " + model, comp);

        return comp;
    }



    public static void checkFilesAreEqual(Path generationPath, Path resultsPath, List<String> fileNames) {
        for (String fileName : fileNames){
            File genFile = new File(generationPath.toString() + "/" + fileName);
            File fileTarget = new File(resultsPath.toString() + "/" + fileName);
            assertTrue(areBothFilesEqual(genFile, fileTarget));
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
}
