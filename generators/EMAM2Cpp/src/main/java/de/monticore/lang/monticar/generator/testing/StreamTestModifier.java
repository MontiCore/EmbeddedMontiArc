/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.testing;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

/**
 */
public class StreamTestModifier {

    public static void updateStreamTestWithResults(String filePathStreamTest, String filePathResults) {
        List<String> streamTestLines = readLines(filePathStreamTest);
        streamTestLines.remove(streamTestLines.size() - 1);
        streamTestLines.addAll(filterOutput(readLines(filePathResults)));
        streamTestLines.add("}");

        writeLines(filePathStreamTest, streamTestLines);
    }

    public static List<String> filterOutput(List<String> content) {
        List<String> filteredContent = new ArrayList<>();
        List<PortNameValues> portNameValuesList = new ArrayList<>();
        for (String line : content) {
            String[] splits = line.split("\\:");
            String portName = splits[0];
            String value = splits[1];
            boolean addNew = true;
            for (PortNameValues portNameValues : portNameValuesList) {
                if (portNameValues.getPortName().equals(portName)) {
                    addNew = false;
                    portNameValues.addValue(value);
                }
            }
            if (addNew) {
                portNameValuesList.add(new PortNameValues(portName, value));
            }
        }

        for (PortNameValues portNameValues : portNameValuesList) {
            StringBuilder result = new StringBuilder();
            result.append(portNameValues.getPortName());
            result.append(": ");
            for (int i = 0; i < portNameValues.getValues().size(); ++i) {
                String value = portNameValues.getValues().get(i);
                result.append(value);
                if (i + 1 < portNameValues.getValues().size()) {
                    result.append(" tick ");
                }
            }
            result.append(";\n");

            filteredContent.add(result.toString());
        }
        return filteredContent;
    }

    public static void writeLines(String filePath, List<String> content) {
        File streamTest = new File(filePath);
        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter(streamTest));
            for (String line : content) {
                writer.write(line);
                writer.write("\n");
            }
            writer.close();
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    public static List<String> readLines(String filePath) {
        File streamTest = new File(filePath);
        if (!streamTest.exists()) {

            File curDir = new File(OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/exec/");
            File curDir2 = new File(OSHelper.getDirPrefix() + "/target/generated-sources-cpp/streamtest/");
//            System.out.println("Directories:");
            getAllFiles(curDir);
            getAllFiles(curDir2);
        }
        List<String> lines = new ArrayList<>();
        try {
            BufferedReader reader = new BufferedReader(new FileReader(streamTest));
            String nextLine;
            while ((nextLine = reader.readLine()) != null) {
                lines.add(nextLine);
            }
            reader.close();
        } catch (Exception ex) {
            ex.printStackTrace();
        }
        return lines;
    }

    public static void getAllFiles(File curDir) {
        File[] filesList = curDir.listFiles();
        for (File f : filesList) {
//            System.out.println(f.getName());
        }
    }
}
