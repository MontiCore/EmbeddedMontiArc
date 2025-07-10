/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.tools;

import java.io.File;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class SearchFiles {

    public static List<File> searchFiles(File path, String... fileType) {
        SearchFiles tc = new SearchFiles();
        List<File> result = new LinkedList<>();
        Set<String> fileTypes = new HashSet<>();
        for (String type : fileType)
            fileTypes.add(type);
        tc.walk(path, fileTypes, result);
        return result;
    }

    public void walk(File root, Set<String> fileTypes, List<File> result) {
        File[] list = root.listFiles();

        if (list == null) return;

        for (File file : list) {
            if (file.isDirectory())
                walk(file, fileTypes, result);
            else
                for (String fileType : fileTypes)
                    if (file.getName().endsWith("." + fileType))
                        result.add(file);
        }
    }
}
