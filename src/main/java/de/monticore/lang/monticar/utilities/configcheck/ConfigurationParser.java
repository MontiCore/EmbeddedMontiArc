package de.monticore.lang.monticar.utilities.configcheck;

import com.clust4j.utils.EntryPair;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class ConfigurationParser {
    public static Map<String, String> parseConfiguration(TrainingConfiguration trainingConfiguration) {
        File configCheckPath = trainingConfiguration.getPathToProject();
        String backend = trainingConfiguration.getBackend().name();
        String modelToTrain = trainingConfiguration.getModelToTrain();

        if (!configCheckPath.isDirectory()) {
            return new TreeMap<>();
        }

        List<File> confFiles = findConfFiles(configCheckPath);
        Map<String, String> configurationMap = parseConfFiles(confFiles);
        configurationMap.put("backend", backend);
        configurationMap.put("modelToTrain", modelToTrain);

        return configurationMap;
    }

    private static List<File> findConfFiles(File directory) {
        File[] files = directory.listFiles();
        List<File> confFiles = new ArrayList<>();

        if (files != null) {
            for (File file : files) {
                if (file.isDirectory()) {
                    confFiles.addAll(findConfFiles(file));
                } else {
                    if (file.getName().toLowerCase().endsWith(".conf")) {
                        confFiles.add(file);
                    }
                }
            }
        }
        return confFiles;
    }

    private static Map<String, String> parseConfFiles(List<File> confFiles) {
        Map<String, String> configurationMap = new TreeMap<>();
        for (File confFile : confFiles) {
            try {
                Map<String, String> conf = parseConfFile(confFile);
                configurationMap.putAll(conf);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        return configurationMap;
    }

    private static Map<String, String> parseConfFile(File confFile) throws IOException {
        TreeMap<String, String> configMap = new TreeMap<>();
        BufferedReader br = new BufferedReader(new FileReader(confFile));
        String line;
        String nestedKey = "";
        boolean nested = false;

        while ((line = br.readLine()) != null) {
            line = line.replace("{}", "").trim();
            if (!line.isEmpty() && !line.startsWith("/*")) {
                if (line.endsWith("{")) {
                    line = line.substring(0, line.length() - 1).trim();
                    Map.Entry<String, String> entry = parseLine(line);
                    if (entry != null) {
                        configMap.put(entry.getKey(), entry.getValue());
                        nestedKey = entry.getValue();
                        nested = true;
                    }
                } else if (line.endsWith("}")) {
                    nested = false;
                } else {
                    Map.Entry<String, String> entry = parseLine(line);
                    if (entry != null) {
                        if (nested && !nestedKey.isEmpty()) {
                            configMap.put(String.format("%s_%s", nestedKey, entry.getKey()), entry.getValue());
                        } else {
                            configMap.put(entry.getKey(), entry.getValue());
                        }
                    }
                }
            }
        }

        br.close();
        return configMap;
    }

    private static Map.Entry<String, String> parseLine(String line) {
        String[] parts = line.split(":");
        if (parts.length == 2) {
            String key = parts[0].trim();
            String value = parts[1].trim();
            return new EntryPair<>(key, value);
        }
        return null;
    }
}
