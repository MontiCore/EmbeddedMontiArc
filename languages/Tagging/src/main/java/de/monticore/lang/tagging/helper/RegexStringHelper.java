/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.tagging.helper;

import javax.measure.quantity.Quantity;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class RegexStringHelper {

    public static String getMatcher(String input) {
        input = input.replaceFirst("\\{\\s*", "");
        input = input.replaceFirst("\\s*}$", "");
        input = input.replaceFirst("^\\s+", "");

        Matcher m = Pattern.compile("\\$\\{(\\w+):(\\w+)\\}").matcher(input);
        while (m.find()) {
            input = input.replace(m.group(0), handleVarGroup(m.group(2)));
        }

        return input;
    }

    private static String handleVarGroup(String type) {
        String ret = "";

        if (type.equals("String")) {
            ret += "(\\\\w+";
        } else {
            ret += "(\\\\d+(?:\\\\.\\\\d+)?(?:n|m|c|d|h|k|M|G|T)?";
            ret += getUnit(type);
            ret += ")";
        }

        ret += "\\\\s*";
        return ret;
    }

    private static String getUnit(String type) {
        switch (type) {
            case "Length":
                return "m";
            case "Mass":
                return "g";
            case "Duration":
                return "s";
            case "ElectricCurrent":
                return "A";
            case "Temperature":
                return "(?:K|C|F)";
            default:
                return null;
        }
    }
}
