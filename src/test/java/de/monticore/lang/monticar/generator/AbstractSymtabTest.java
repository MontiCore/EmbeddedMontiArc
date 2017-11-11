/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
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
            System.out.println("" + fileTarget.exists() + "Exists:");
            System.out.println(f.getName() + " " + fileTarget.getName() + "Comparing:");
            assertTrue(areBothFilesEqual(f, fileTarget));
        }
    }

    public static boolean areBothFilesEqual(File file1, File file2) {
        if (!file1.exists() || !file2.exists()) {
            return false;
        }
        List<String> lines1;
        List<String> lines2;
        try {
            lines1 = Files.readAllLines(file1.toPath());
            lines2 = Files.readAllLines(file2.toPath());
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
        lines1 = discardEmptyLines(lines1);
        lines2 = discardEmptyLines(lines2);
        if (lines1.size() != lines2.size()) {
            return false;
        }
        int len = lines1.size();
        for (int i = 0; i < len; i++) {
            if (!lines1.get(i).equals(lines2.get(i))) {
                return false;
            }
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
