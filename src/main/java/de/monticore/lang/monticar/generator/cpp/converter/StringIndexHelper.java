package de.monticore.lang.monticar.generator.cpp.converter;

/**
 * @author Sascha Schneiders
 */
public class StringIndexHelper {

    public static String modifyContentBetweenBracketsByAdding(String input, String modifier) {
        String result = "";
        boolean done = false;
        int indexFirst = input.indexOf("(", 0);
        if (indexFirst == -1)
            return input;
        result += input.substring(0, indexFirst);
        int indexSecond = input.indexOf(",", indexFirst + 1);
        if (indexSecond == -1) {
            indexSecond = input.indexOf(")", indexFirst + 1);
            done = true;
        }
        while (!done) {
            result += input.substring(indexFirst, indexSecond) + modifier;
            indexFirst = indexSecond;
            indexSecond = input.indexOf(",", indexSecond + 1);

            if (indexSecond == -1) {
                indexSecond = input.indexOf(")", indexFirst + 1);
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
            result+=newEndPart;
        }
        return result;
    }

}
