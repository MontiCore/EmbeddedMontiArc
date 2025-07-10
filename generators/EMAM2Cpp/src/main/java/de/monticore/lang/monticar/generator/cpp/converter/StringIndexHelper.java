/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

/**
 */
public class StringIndexHelper {

    public static String modifyContentBetweenBracketsByAdding(String input, String modifier) {
        return modifyContentBetweenBracketsByAdding(input, modifier, "(", ",", ")");
    }

    /**
     * Only adds modifier in the first occurring bracket
     */
    public static String modifyContentBetweenBracketsByAdding(String input, String modifier, String bracketStart, String bracketSep, String bracketEnd) {
        String result = "";
        boolean done = false;
        int indexFirst = input.indexOf(bracketStart, 0);
        if (indexFirst == -1)
            return input;
        result += input.substring(0, indexFirst);
        int indexSecond = input.indexOf(bracketSep, indexFirst + bracketSep.length());
        if (indexSecond == -1) {
            indexSecond = input.indexOf(bracketEnd, indexFirst + bracketEnd.length());
            done = true;
        }
        while (!done) {
            result += input.substring(indexFirst, indexSecond) + modifier;
            indexFirst = indexSecond;
            indexSecond = input.indexOf(bracketSep, indexSecond + bracketSep.length());

            if (indexSecond == -1) {
                indexSecond = input.indexOf(bracketEnd, indexFirst + bracketEnd.length());
                done = true;
            }
        }
        result += input.substring(indexFirst, indexSecond) + modifier;
        result += input.substring(indexSecond);
        return result;
    }

    public static String modifyContentBetweenBracketsByRemoving(String input, String endPart, String newEndPart) {
        String result = input;
        if (input.endsWith(endPart)) {
            int indexFirst = input.lastIndexOf(endPart);
            result = input.substring(0, indexFirst);
            result += newEndPart;
        }
        return result;
    }

    public static String modifyContentBetweenBracketsByAddingForAllBrackets(String input, String modifier, String bracketStart, String bracketSep, String bracketEnd) {
        String[] parts = input.split("(?<=\\))");
        StringBuilder sb = new StringBuilder();
        for (String s : parts) {
            sb.append(modifyContentBetweenBracketsByAdding(s, modifier, bracketStart, bracketSep, bracketEnd));
        }
        return sb.toString();
    }

    public static String modifyContentBetweenBracketsByAddingForAllBrackets(String input, String modifier) {
        return modifyContentBetweenBracketsByAddingForAllBrackets(input, modifier, "(", ",", ")");
    }

}
