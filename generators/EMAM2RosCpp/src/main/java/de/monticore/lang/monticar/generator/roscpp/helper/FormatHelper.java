/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp.helper;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class FormatHelper {

    private FormatHelper() {
    }

    public static String fixIndentation(String lines) {
        return fixIndentation(Arrays.asList(lines.split("\\n"))).stream()
                .collect(Collectors.joining("\n"));
    }

    public static List<String> fixIndentation(List<String> lines) {
        ArrayList<String> res = new ArrayList<>();
        int depth = 0;
        for (String line : lines) {
            if (line.contains("}") && depth > 0)
                depth--;
            StringBuilder indentString = new StringBuilder();
            for (int i = 0; i < depth; i++)
                indentString.append("\t");

            res.add(indentString.toString() + line.trim());

            if (line.contains("{"))
                depth++;
        }
        return res;
    }
}
