package de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper;

import java.io.File;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class SearchFiles {
    public static void main(String[] args) {
        File project = new File("C:\\Praktikum\\Neu\\EmbeddedMontiArcMath");
        String fileType = "emam";
        List<File> files = searchFiles(project, fileType);
        for(File file: files){
            System.out.println("File: " + file.getAbsoluteFile());
        }
    }

    public static List<File> searchFiles(File path, String... fileType){
        SearchFiles tc = new SearchFiles();
        List<File> result = new LinkedList<>();
        Set<String> fileTypes = new HashSet<>();
        for(String type: fileType)
            fileTypes.add(type);
        tc.walk(path, fileTypes, result);
        return result;
    }

    public void walk( File root, Set<String> fileTypes, List<File> result) {
        File[] list = root.listFiles();

        if (list == null) return;

        for (File file : list) {
            if (file.isDirectory()) {
                walk(file, fileTypes, result);
            } else {
                if(file.getName().contains("Autopilot")){
                    int i = 0;
                }
                for(String fileType: fileTypes)
                    if (file.getName().endsWith("." + fileType))
                        result.add(file);
            }
        }
    }
}
