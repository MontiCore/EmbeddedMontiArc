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
package de.monticore.lang.monticar.generator.cpp.converter;

/**
 * @author Sascha Schneiders
 * @author Christoph Richter
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
