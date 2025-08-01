/* (c) https://github.com/MontiCore/monticore */
package de.monitcore.lang.monticar.utilities.tools;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.*;

public class SearchFiles {

    public static List<File> searchFiles(String path, String... fileType){
        return SearchFiles.searchFiles(new File(path), fileType);
    }

    public static List<File> searchFiles(File path, String... fileType) {
//        List<File> result = new LinkedList<File>();
        Set<String> fileTypes = new HashSet<String>();
        for (String type : fileType)
            fileTypes.add(type);
//        SearchFiles.walk(path, fileTypes, result);
        return searchFiles(path, fileTypes);
    }

    public static List<File> searchFiles(File path, Set<String> fileTypes){
        List<File> result = new LinkedList<File>();
        SearchFiles.walk(path, fileTypes, result);
        return result;
    }

    public static Map<String, File> searchFilesMap(String path, String... fileType){
        Set<String> fileTypes = new HashSet<String>();
        for (String type : fileType)
            fileTypes.add(type);
        return searchFilesMap(new File(path), fileTypes);
    }
    public static Map<String, File> searchFilesMap(String path, Set<String> fileTypes){
        return searchFilesMap(new File(path), fileTypes);
    }

    public static Map<String, File> searchFilesMap(File path, Set<String> fileTypes){
        List<File> lf = searchFiles(path, fileTypes);
        Map<String, File> res = new HashMap<String, File>();
        for (File f: lf) {
            //try {
                res.put(path.toURI().relativize(f.toURI()).getPath(), f);
                //res.put(f.getCanonicalPath().replaceFirst(path.getCanonicalPath(), ""), f);
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
        }
        return res;
    }



    public static void walk(File root, Set<String> fileTypes, List<File> result) {

        File[] list = root.listFiles();

        if (list == null) return;

        for (File file : list) {
            if (file.isDirectory()) {
                walk(file, fileTypes, result);
            } else {
                for (String fileType : fileTypes)
                    if (file.getName().endsWith("." + fileType))
                        result.add(file);
            }
        }
    }


    public static String hashDirFiles(String path){
        List<File> files = new ArrayList<>();
        walkAll(Paths.get(path).toFile(), files);

        StringBuilder sb = new StringBuilder();
        for (File f : files) {

            try {
                sb.append(ChecksumChecker.getChecksumForFileMD5(f.getAbsolutePath()));
                sb.append(ChecksumChecker.getChecksumForStringMD5(f.getAbsolutePath()));
            } catch (IOException e) {
                sb.append("--");
            }
        }

        return sb.toString();
    }


    public static void walkAll(File root, List<File> result){
        File[] list = root.listFiles();
        if (list == null) return;
        for (File file : list) {
            if (file.isDirectory()) {
                walkAll(file, result);
            } else {
                result.add(file);
            }
        }
    }
}

