package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicEventHandlerInstanceSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.instruction.EventConnectInstructionCPP;
import de.monticore.lang.monticar.generator.cpp.instruction.ExecuteDynamicConnects;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class EventDynamicConnectConverter {

    protected static final String DYNPORTID = "_%s_dynPortID";
    protected static final String THISCOMPONENTPORTREQUEST = "int "+DYNPORTID+" = __%s_connect_request.front(); __%s_connect_request.pop();\n";
    protected static final String DYNPORTIDININSTANCE = "_%s_%s_dynPortID";
    protected static final String DYNPORTIDININSTANCEINIT = "int "+DYNPORTIDININSTANCE+" = -1;\n";

    public static boolean generateDynamicConnectEvent(EMADynamicEventHandlerInstanceSymbol event, EMAComponentInstanceSymbol componentSymbol, Method executeMethod, BluePrintCPP bluePrint ) {

        List<String> names= new ArrayList<>();
        event.getCondition().getConnectPortNames(names);
        java.util.Collections.sort(names);


        List<String> newInstances = getNewInstances(event);
        Map<String,List<String>> newPortsInstances = getNewPortsOfInstances(event);

        generateConnectMethod(names, componentSymbol, bluePrint);

        String bodyname = "__event_body_"+event.getName().replace("[", "_").replace("]", "_");
        Method body = new Method(bodyname, "void");
        body.setPublic(false);
        generateDynamicMethod(bluePrint, bodyname);


        body.addInstruction(new TargetCodeInstruction("while("+EventConnectInstructionCPP.getEventNameCPP(event.getName())+"()){\n"));

        generateHandleConnectRequestsOfQueues(names, body);
        generateHandleConnectRequestInInstances(newPortsInstances, body);


        generateConnects(event, body, bluePrint, executeMethod);

        body.addInstruction(new TargetCodeInstruction("}\n"));
        bluePrint.addMethod(body);
        return true;
    }

    protected static void generateConnectMethod(List<String> names, EMAComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint){

        String name = "connect_"+String.join("_", names);
        if(bluePrint.getMethod(name).isPresent()){
            return;
        }

        Log.info("Create connect method: "+name+"(...)", "EventConverter");

        Method method = new Method(name, "bool");
        List<String> checks = new ArrayList<>();
        for(String n : names ){
            Variable v = new Variable();
            v.setName(n+"_indexref");
            v.setTypeNameTargetLanguage("int*");
            method.addParameter(v);

            long counter = componentSymbol.getPortInstanceList().stream().filter(p->p.getNameWithoutArrayBracketPart().equals(n)).count();

            String inst = String.format("*%s_indexref = dynamicconnect(%d, __%s_connected, &__%s_connect_request);\n", n, counter, n,n);

            method.addInstruction(
                    new TargetCodeInstruction(inst)
            );

            checks.add(String.format("(*%s_indexref < 0)", n));
        }
        method.addInstruction(new TargetCodeInstruction(
                String.format("if(%s){return false;}\n", String.join(" || ", checks))
        ));
        method.addInstruction(new TargetCodeInstruction("return true;\n"));
        bluePrint.addAdditionalIncludeString("DynamicHelper");
        bluePrint.addMethod(method);
    }

    protected static void generateDynamicMethod(BluePrintCPP bluePrint, String eventBodyName){
        Optional<Method> dynamic = bluePrint.getMethod("dynamic");
        if(!dynamic.isPresent()){
            Method d = new Method("dynamic", "void");
            d.setPublic(false);
            bluePrint.addMethod(d);
            dynamic = Optional.of(d);
        }
        dynamic.get().addInstruction(new TargetCodeInstruction(eventBodyName+"();\n"));
    }

    protected static List<String> getNewInstances(EMADynamicEventHandlerInstanceSymbol event){
        Set<String> insts = new HashSet<>();
        for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){
            Optional<String> comp = connector.getSourceComponentName();
            if(connector.isDynamicSourceNewComponent() && comp.isPresent()){
                insts.add(EMAPortSymbol.getNameWithoutArrayBracketPart(comp.get()));
            }

            comp = connector.getTargetComponentName();
            if(connector.isDynamicTargetNewComponent() && comp.isPresent()){
                insts.add(EMAPortSymbol.getNameWithoutArrayBracketPart(comp.get()));
            }
        }
        return new ArrayList<>(insts);
    }

    protected static Map<String, List<String>> getNewPortsOfInstances(EMADynamicEventHandlerInstanceSymbol event){
        Map<String,List<String>> newPorts = new HashMap<>();
        for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){
            Optional<String> comp = connector.getSourceComponentName();
            String p;
            if( connector.isDynamicSourceNewPort() && comp.isPresent()){
                if(!newPorts.containsKey(comp.get())){
                    newPorts.put(comp.get(), new ArrayList<>());
                }
                p = EMAPortSymbol.getNameWithoutArrayBracketPart(connector.getSourcePortName());
                if(!newPorts.get(comp.get()).contains(p)){
                    newPorts.get(comp.get()).add(p);
                }
            }

            comp = connector.getTargetComponentName();
            if(connector.isDynamicTargetNewPort() && comp.isPresent()){
                if(!newPorts.containsKey(comp.get())){
                    newPorts.put(comp.get(), new ArrayList<>());
                }
                p = EMAPortSymbol.getNameWithoutArrayBracketPart(connector.getTargetPortName());
                if(!newPorts.get(comp.get()).contains(p)){
                    newPorts.get(comp.get()).add(p);
                }
            }
        }
        return newPorts;
    }

    protected static void generateHandleConnectRequestsOfQueues(List<String> names, Method body){
        for(String name : names){
            body.addInstruction(new TargetCodeInstruction(
                    String.format(THISCOMPONENTPORTREQUEST,
                            name, name, name)));
        }
    }

    protected static void generateHandleConnectRequestInInstances(Map<String, List<String>> newPorts, Method body){
        for (Map.Entry<String,List<String>> entry : newPorts.entrySet()){
            Collections.sort(entry.getValue());
            String inst = entry.getKey()+".connect_"+String.join("_", entry.getValue())+"(";
//            for(String port : entry.getValue()){
            for(int i = 0; i < entry.getValue().size(); ++i){

                body.addInstruction(new TargetCodeInstruction(String.format(
                        DYNPORTIDININSTANCEINIT, convertName(entry.getKey()), entry.getValue().get(i)
                )));

                inst = inst + "&"+String.format(DYNPORTIDININSTANCE, convertName(entry.getKey()), entry.getValue().get(i));
                if(i < entry.getValue().size()-1){
                    inst += ", ";
                }
            }

            body.addInstruction(new TargetCodeInstruction(String.format(
                    "if(!"+inst+")){return ;}\n"
            )));

        }
    }

    protected static void generateConnects(EMADynamicEventHandlerInstanceSymbol event, Method body, BluePrintCPP bluePrint, Method executeMethod){
        for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){

            Optional<String> before = Optional.empty();
            String sourceName = connector.getSource();
            String targetName = connector.getTarget();
            EMAPortInstanceSymbol target = connector.getTargetPort();

            if(connector.isDynamicSourceNewPort()){
                sourceName = generateConnectNameForNewPort(sourceName, connector.getSourceComponentName(), connector.getSourcePortName());
//                afterComponent = connector.getSourceComponentName();

            }else{
                sourceName = GeneralHelperMethods.getTargetLanguageVariableInstanceName(sourceName);
            }

            if(connector.isDynamicTargetNewPort()){
                targetName = generateConnectNameForNewPort(targetName, connector.getTargetComponentName(), connector.getTargetPortName());
                before = connector.getTargetComponentName();
            }else{
                targetName = GeneralHelperMethods.getTargetLanguageVariableInstanceName(targetName);
                before = connector.getTargetComponentName();
            }


            Optional<VariableType> vt = TypeConverter.getVariableTypeForMontiCarTypeName(target.getTypeReference().getName());
            generateEventDynamicConnectVector(vt.get().getTypeNameTargetLanguage(), bluePrint);

            executeMethod.addInstruction(new ExecuteDynamicConnects(before));

            if(!before.isPresent()){
                before = Optional.of("NULL");
            }else{
                before = Optional.of("&"+GeneralHelperMethods.getTargetLanguageVariableInstanceName(before.get()));
            }
            body.addInstruction(new TargetCodeInstruction(String.format(
                    "__dynamic_%s_connect.push_back({%s, &%s, &%s});\n", vt.get().getTypeNameTargetLanguage(), before.get(), sourceName, targetName
            )));


        }
    }

    protected static void generateEventDynamicConnectVector(String typeName, BluePrint bluePrint){

        Optional<Method> execDynConnects = bluePrint.getMethod("executeDynamicConnects");
        if(!execDynConnects.isPresent()){
            Method m = new Method("executeDynamicConnects", "void");
            Variable p = new Variable();
            p.setName("afterComponent");
            p.setTypeNameTargetLanguage("void*");
            m.addParameter(p);

            bluePrint.addMethod(m);
            execDynConnects = Optional.of(m);
        }

        String cConnectVarName = "__dynamic_"+typeName+"_connect";
        if(!bluePrint.getVariable(cConnectVarName).isPresent()){
            Variable vdConnect = new Variable();
            vdConnect.setName(cConnectVarName);
            vdConnect.setTypeNameTargetLanguage("std::vector<connection<"+typeName+">>");
            vdConnect.setPublic(false);
            bluePrint.addVariable(vdConnect);

            execDynConnects.get().addInstruction(new TargetCodeInstruction(String.format(
                    "for (std::vector<connection<%s>>::iterator it = __dynamic_%s_connect.begin(); it < __dynamic_%s_connect.end(); ++it) {if(it->afterComponent == afterComponent){*(*it).target = *(*it).source;}}\n",
                    typeName, typeName,typeName
            )));
        }
    }

    protected static String generateConnectNameForNewPort(String allName, Optional<String> componenName, String portName){
        String result = "";
        if(componenName.isPresent()){
            result = String.format("%s["+DYNPORTIDININSTANCE+"]",
                    GeneralHelperMethods.getTargetLanguageVariableInstanceName(EMAPortSymbol.getNameWithoutArrayBracketPart(allName)),
                    convertName(componenName.get()),
                    EMAPortSymbol.getNameWithoutArrayBracketPart(portName));
        }else{
            result = EMAPortSymbol.getNameWithoutArrayBracketPart(allName);
            result = String.format("%s["+DYNPORTID+"]", result, result);
        }
        return result;
    }

    protected static String convertName(String name){
        return name.replace("[", "_").replace("]", "_");
    }
}
