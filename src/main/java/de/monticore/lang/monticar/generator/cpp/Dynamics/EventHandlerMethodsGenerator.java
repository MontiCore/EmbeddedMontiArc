package de.monticore.lang.monticar.generator.cpp.Dynamics;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicEventHandlerInstanceSymbol;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;

import java.util.Collection;
import java.util.List;


public class EventHandlerMethodsGenerator {

    public static final String EventhandlerIN = "eventhandler_in";
    public static final String EventhandlerOUT = "eventhandler_out";
    public static final String EventHandleMethodExtIN = "_in";
    public static final String EventHandleMethodExtOUT = "_out";
    public static final String EventHandleMethodExtCOND = "_condition";

    public static void generateMethods(EMADynamicComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint){

        Collection<EMADynamicEventHandlerInstanceSymbol> eventHandlers = componentSymbol.getEventHandlers();
        if(eventHandlers.size() == 0){
            return;
        }

        Method inGather = new Method(EventhandlerIN, "void");
        inGather.setPublic(false);

        Method outGather = new Method(EventhandlerOUT, "void");
        outGather.setPublic(false);

        for (EMADynamicEventHandlerInstanceSymbol eh : eventHandlers){
            Method evCond = new Method(eh.getName()+EventHandleMethodExtCOND, "inline bool");
            evCond.setPublic(false);

            Method evIn = new Method(eh.getName()+EventHandleMethodExtIN, "inline void");
            evIn.setPublic(false);

            Method evOut = new Method(eh.getName()+EventHandleMethodExtIN, "inline void");
            evOut.setPublic(false);

        }
        bluePrint.addMethod(inGather);
        bluePrint.addMethod(outGather);
    }

    public static void generateInputsOfEventHandlers(EMAComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint, Method executeMethod){

        if(!(componentSymbol instanceof EMADynamicComponentInstanceSymbol)){
            return;
        }
        EMADynamicComponentInstanceSymbol dynComp = (EMADynamicComponentInstanceSymbol)componentSymbol;

        //TODO: better check
        Collection<EMADynamicEventHandlerInstanceSymbol> eventHandlers = dynComp.getEventHandlers();
        if(eventHandlers.size() == 0){
            return;
        }

        Method inGather = new Method(EventhandlerIN, "void");
        inGather.setPublic(false);
        executeMethod.addInstruction(new TargetCodeInstruction(EventhandlerIN+"();\n"));

        bluePrint.addMethod(inGather);
    }

}
