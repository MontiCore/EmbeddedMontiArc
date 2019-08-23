/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.mqtt;


import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.monticar.Utils;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
import de.monticore.lang.monticar.streamunits._symboltable.StreamUnitsLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import org.junit.Assert;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.Assert.assertTrue;

/**
 * Common methods for symboltable tests
 */
public class AbstractSymtabTest {

    public AbstractSymtabTest() {
    }

    public static TaggingResolver createSymTabAndTaggingResolver(String... modelPath) {
        Scope scope = createSymTab(modelPath);
        TaggingResolver taggingResolver = new TaggingResolver(scope, Arrays.asList(modelPath));
        RosToEmamTagSchema.registerTagTypes(taggingResolver);
        return taggingResolver;
    }

    public static Scope createSymTab(String... modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        EmbeddedMontiArcMathLanguage montiArcLanguage = new EmbeddedMontiArcMathLanguage();
        fam.addModelingLanguage(montiArcLanguage);
        fam.addModelingLanguage(new StreamUnitsLanguage());
        fam.addModelingLanguage(new StructLanguage());
        fam.addModelingLanguage(new EnumLangLanguage());
        ModelPath mp = new ModelPath(new Path[0]);
        String[] var4 = modelPath;
        int var5 = modelPath.length;

        for(int var6 = 0; var6 < var5; ++var6) {
            String m = var4[var6];
            mp.addEntry(Paths.get(m));
        }

        LogConfig.init();
        GlobalScope scope = new GlobalScope(mp, fam);
        Utils.addBuiltInTypes(scope);
        return scope;
    }

    public static void testFilesAreEqual(List<File> files, String restPath) {
        assertTrue(files.size() > 0);
        for (File f : files) {
            File fileTarget = new File("./src/test/resources/results/" + restPath + f.getName());
//            System.out.println("" + fileTarget.exists() + "Exists:");
//            System.out.println(f.getName() + " " + fileTarget.getName() + "Comparing:");
            assertTrue(areBothFilesEqual(f, fileTarget));
        }
    }

    public static void testFilesAreEqual(List<File> files, String relResultPath, String basePath) {
        String resultPath = "src/test/resources/results/";
        for (File f : files) {
            File tmpFile = new File(basePath);
            String relativePath = f.getAbsolutePath().replace(tmpFile.getAbsolutePath(), "");
            File fileTarget = new File(resultPath + relResultPath + relativePath);
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
        lines1 = discardCommentLines(lines1);
        lines2 = discardEmptyLines(lines2);
        lines2 = discardCommentLines(lines2);
        if (lines1.size() != lines2.size()) {
            Assert.fail(
                    "files have different number of lines: "
                            + file1.getAbsolutePath()
                            + " has " + lines1.size()
                            + " lines and " + file2.getAbsolutePath() + " has " + lines2.size() + " lines"
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

    private static List<String> discardCommentLines(List<String> lines) {
        return lines.stream()
                .map(String::trim)
                .filter(l -> !l.startsWith("//"))
                .collect(Collectors.toList());
    }
}

