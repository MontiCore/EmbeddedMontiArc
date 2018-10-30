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

/**
 * @author Sascha Schneiders
 */
public class ConstantConnectInstructionCPP extends ConnectInstruction {

    public ConstantConnectInstructionCPP(Variable variable1, Variable constantVariable) {
        super(variable1, constantVariable);
    }


    @Override
    public String getTargetLanguageInstruction() {
        String resultString = "";
        resultString += "this->";
        resultString += getVariable1().getNameTargetLanguageFormat();
        resultString += " = ";

        resultString += getVariable2().getConstantValue() + ";\n";
        return resultString;
    }
}
