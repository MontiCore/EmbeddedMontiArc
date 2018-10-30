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
