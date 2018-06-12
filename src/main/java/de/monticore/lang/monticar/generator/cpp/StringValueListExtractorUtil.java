package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverterMethodGeneration;
import de.se_rwth.commons.logging.Log;

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

    public static boolean containsPortName(String input) {
        Log.info("trying containsPortName" + input, "Info");

        try {
            //if ((mathExpressionSymbol.isMatrixExpression() && ((MathMatrixExpressionSymbol) mathExpressionSymbol).isMatrixNameExpression()))
            {
                //MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;
                //String fullName = mathMatrixNameExpressionSymbol.getTextualRepresentation();
                String fullName = input;
                while (fullName.length() > 0) {
                    fullName = MathCommandRegisterCPP.removeTrailingStrings(fullName, "(");
                    String name = MathCommandRegisterCPP.calculateName(fullName);
                    Log.info("" + input + " name: " + name, "containsCommandExpression");
                    if (ComponentConverterMethodGeneration.currentComponentSymbol.getPort(name).isPresent()) {
                        return true;
                    }
                    fullName = fullName.substring(name.length() + 1);
                }
            }
        } catch (Exception ex) {
            ex.printStackTrace();
        }
        return false;
    }

}
