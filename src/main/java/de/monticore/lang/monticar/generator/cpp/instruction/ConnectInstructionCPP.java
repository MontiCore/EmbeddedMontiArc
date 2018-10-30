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
package de.monticore.lang.monticar.generator.cpp.instruction;

import de.monticore.lang.monticar.generator.ConnectInstruction;
import de.monticore.lang.monticar.generator.Variable;
import de.se_rwth.commons.logging.Log;

/**
 * @author Sascha Schneiders
 */
public class ConnectInstructionCPP extends ConnectInstruction {
    public ConnectInstructionCPP(Variable variable1, Variable variable2) {
        super(variable1, variable2);
    }



    @Override
    public String getTargetLanguageInstruction() {
        String resultString = "";

        if (getVariable1().isArray()&&false) {
            Log.info("Size: "+getVariable1().getArraySize(),"Array True Method:");
            for (int i = 0; i < getVariable1().getArraySize(); ++i) {
                if (isUseThis1())
                    resultString += "this->";
                resultString += getVariable1().getNameTargetLanguageFormat() + "[" + i + "]";
                resultString += " = ";
                if (isUseThis2()) {
                    resultString += "this->";
                }
                resultString += getVariable2().getNameTargetLanguageFormat() + "[" + i + "]" + ";\n";
            }
        } else {
            Log.info("var1: "+getVariable1().getName()+" var2: "+getVariable2().getName(),"Array False Method:");
            if (isUseThis1())
                resultString += "this->";
            resultString += getVariable1().getNameTargetLanguageFormat();
            resultString += " = ";
            if (isUseThis2()) {
                resultString += "this->";
            }
            resultString += getVariable2().getNameTargetLanguageFormat() + ";\n";
            Log.info(resultString,"ResultString:");
        }
        return resultString;
    }
}
