/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.instruction;

import de.monticore.lang.monticar.generator.ConnectInstruction;
import de.monticore.lang.monticar.generator.Variable;

/**
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
