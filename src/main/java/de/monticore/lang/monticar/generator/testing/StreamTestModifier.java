/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.generator.testing;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
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
