/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.instruction;

import de.monticore.lang.monticar.generator.ConnectInstruction;
import de.monticore.lang.monticar.generator.Variable;
import de.se_rwth.commons.logging.Log;

/**
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
