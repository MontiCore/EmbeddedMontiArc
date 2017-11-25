package de.monticore.lang.monticar.generator.cpp;

/**
 * @author Sascha Schneiders
 */
public class StringValueListExtractorUtil {

    /**
     * Method does not check if valueListString contains the required amount of elements
     *
     * @param valueListString
     * @param element
     * @return
     */
    public static String getElement(String valueListString, int element, String separator) {
        valueListString = valueListString.replaceAll("\\(", "").replaceAll("\\(", "");
        int index = valueListString.indexOf(separator);
        int lastIndex = 0;
        for (int i = 0; i < element; ++i) {
            lastIndex = index + 1;
            index = valueListString.indexOf(separator, index + 1);
        }
        if (index == -1) {
            index = valueListString.length() - 1;
        }
        valueListString = valueListString.substring(lastIndex, index);
        return valueListString.replaceAll(" ", "");
    }

    public static String getElement(String valueListString, int element) {
        return getElement(valueListString, element, ",");
    }
}
