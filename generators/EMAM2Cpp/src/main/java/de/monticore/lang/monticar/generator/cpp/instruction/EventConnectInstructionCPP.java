/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.instruction;

import de.monticore.lang.monticar.generator.EventConnectInstruction;
import de.monticore.lang.monticar.generator.Variable;
import de.se_rwth.commons.logging.Log;

public class EventConnectInstructionCPP extends EventConnectInstruction {


    public EventConnectInstructionCPP(String name) {
        super(name);
    }

    public EventConnectInstructionCPP(String name, Variable variable1, Variable variable2) {
        super(name, variable1, variable2);
    }

    public EventConnectInstructionCPP(String name, Variable variable1, boolean useThis1, Variable variable2, boolean useThis2) {
        super(name, variable1, useThis1, variable2, useThis2);
    }


    public String getEventNameCPP(){
        return getEventNameCPP(this.getEventName());
    }

    @Override
    public String getTargetLanguageInstruction() {
        String resultString = "if( "+this.getEventNameCPP()+"() ){ ";

        Log.info("var1: "+getVariable1().getName()+" var2: "+getVariable2().getName(),"Array False Method:");
        if (isUseThis1())
            resultString += "this->";
        resultString += getVariable1().getNameTargetLanguageFormat();
        resultString += " = ";
        if (isUseThis2()) {
            resultString += "this->";
        }
        resultString += getVariable2().getNameTargetLanguageFormat() + ";";
        Log.info(resultString,"ResultString:");

        return resultString+" }\n";

    }

    public static String getEventNameCPP(String name){
        return "__event_condition_"+name.replace("[", "_")
                .replace("]", "_");
    }
}
