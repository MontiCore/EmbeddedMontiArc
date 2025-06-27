/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.EMAMBluePrint;

/**
 */
public class GeneralHelperMethods {


    public static String getTargetLanguageComponentName(String fullName) {
        return fullName.replaceAll("\\.", "_").replaceAll("\\[", "_").replaceAll("\\]", "_");
    }

    public static String replaceUnderScoreWithSquareBrackets(String componentName, String regex, String replacement) {
        return componentName.replaceFirst(regex, replacement);
    }

    public static String getTargetLanguageVariableInstanceName(String componentName, EMAMBluePrint bluePrint) {
        while (!bluePrint.getVariable(componentName).isPresent() && componentName.contains("_")) {
            componentName = replaceUnderScoreWithSquareBrackets(componentName, "\\_", "[");
            componentName = replaceUnderScoreWithSquareBrackets(componentName, "\\_", "]");
        }
        return getTargetLanguageVariableInstanceName(componentName);
    }

    /**
     * fixes array access
     *
     * @param name
     * @return
     */
    public static String getTargetLanguageVariableInstanceName(String name) {


        String nameChanged = "";
        int indexSecond = 0;
        while (true) {
            int indexFirst = name.indexOf("[", indexSecond);
            if (indexFirst != -1) {
                nameChanged += name.substring(indexSecond, indexFirst);
                indexSecond = name.indexOf("]", indexFirst + 1);
                if (indexSecond != -1) {
                    String subString = name.substring(indexFirst + 1, indexSecond++);
                    try {
                        nameChanged += "[" + (Integer.parseInt(subString) - 1) + "]";
                    } catch (Exception ex) {
                        nameChanged += "[" + subString + "- 1]";
                    }
                } else
                    break;
            } else
                break;
        }
        if (indexSecond != -1 && name.length() > nameChanged.length())
            nameChanged += name.substring(indexSecond);
        if (nameChanged.equals(""))
            return name;
        return nameChanged;

    }

    public static String getTargetLanguageQualifiedComponentName(String componentName) {
        if (!componentName.contains(".")) return componentName;
        String[] parts = componentName.split("\\.");
        if (parts.length <= 1) return componentName;
        return String.join("->", parts);
    }

    public static String getTargetLanguageComponentVariableInstanceName(String componentName) {
        return getTargetLanguageComponentName(getTargetLanguageVariableInstanceName(componentName));
    }
}
