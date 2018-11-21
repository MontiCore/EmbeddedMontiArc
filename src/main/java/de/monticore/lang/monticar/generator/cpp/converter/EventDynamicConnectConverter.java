package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicEventHandlerInstanceSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.instruction.EventConnectInstructionCPP;
import de.monticore.lang.monticar.generator.cpp.instruction.ExecuteDynamicConnects;
import de.se_rwth.commons.logging.Log;


import java.util.*;
import java.util.stream.Collectors;

public class EventDynamicConnectConverter {

    protected static final String DYNPORTID = "_%s_dynPortID";
    protected static final String THISCOMPONENTPORTREQUEST = "int "+DYNPORTID+" = __%s_connect_request.front(); __%s_connect_request.pop(); _connected_idxs[%d] = "+DYNPORTID+";\n";


    protected static final String DYNPORTIDININSTANCE = "_%s_%s_dynPortID";
    protected static final String DYNPORTIDININSTANCEINIT = "int "+DYNPORTIDININSTANCE+" = -1;\n";
    protected static final String DYNINSTANCEID = "_%s_dynINSTID";
    protected static final String DYNINSTANCECONNECT = "int "+DYNINSTANCEID+" = dynamicconnect(%d, __%s_connected); if( "+DYNINSTANCEID+" < 0 ){ free(_connected_idxs); return; }  _connected_idxs[%d] = "+DYNINSTANCEID+";\n";

    protected static final String FREE_CONNECTIONTRACARRAY = "__event_connects_%s";
    protected static final String FREE_THISCOMPONENTPORTREQUEST = "int "+DYNPORTID+" = _connected_idxs[%d];\n";
    protected static final String FREE_THISCOMPONENTPORTREQUEST_CONNECTED_ARRAY = "__%s_connected["+DYNPORTID+"] = false;\n";
    protected static final String FREE_DYNINSTANCECONNECT = "int "+DYNINSTANCEID+" = _connected_idxs[%d];\n";
    protected static final String FREE_DYNINSTANCECONNECT_CONNECTED_ARRAY =  "__%s_connected["+DYNINSTANCEID+"] = false;\n";
    protected static final String FREE_DYNPORTIDININSTANCE = "int "+DYNPORTIDININSTANCE+" = _connected_idxs[%d];\n";


    protected static int free_method_index_counter = 0;
    protected static List<String> resetConnectedArray = new ArrayList<>();

    public static boolean generateDynamicConnectEvent(EMADynamicEventHandlerInstanceSymbol event, EMAComponentInstanceSymbol componentSymbol, Method executeMethod, BluePrintCPP bluePrint ) {

        free_method_index_counter = 0;
        resetConnectedArray.clear();

        List<String> names= new ArrayList<>();
        event.getCondition().getConnectPortNames(names);
        java.util.Collections.sort(names);


        List<String> newInstances = getNewInstances(event);
        Map<String,List<String>> newPortsInstances = getNewPortsOfInstances(event);

        generateConnectMethod(names, componentSymbol, bluePrint);
        Method free = generateFreeMethod(names, event, bluePrint);


        String bodyname = "__event_body_"+event.getName().replace("[", "_").replace("]", "_");
        Method body = new Method(bodyname, "void");
        body.setPublic(false);
        generateDynamicMethod(bluePrint, bodyname);


        body.addInstruction(new TargetCodeInstruction("while("+EventConnectInstructionCPP.getEventNameCPP(event.getName())+"()){\n"));

        generateHandleConnectRequestsOfQueues(names, body, free);
        generateHandleConnectRequestsOfDynamicInstances(newInstances, body, componentSymbol, free);
        generateHandleConnectRequestInInstances(newPortsInstances, body, free);


        generateConnects(event, body, bluePrint, executeMethod, free);

//        generateDummyConnects(event, componentSymbol, executeMethod, bluePrint);


        //finish everything that needs to be set
//
        body.getInstructions().add(1, new TargetCodeInstruction("int* _connected_idxs = (int *)calloc("+free_method_index_counter+", sizeof(int));\n"));
        generateEndOfFreeMethod(event, free, componentSymbol);

        String varName = "__event_connects_"+convertName(event.getName());
        if(!bluePrint.getVariable(varName).isPresent()){
            Variable v = new Variable();
            v.setName(varName);
            v.setTypeNameTargetLanguage("std::vector<int*>");
            v.setPublic(false);
            bluePrint.addVariable(v);
        }
        body.addInstruction(new TargetCodeInstruction(varName+".push_back(_connected_idxs);\n"));

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

    protected static Method generateFreeMethod(List<String> names, EMADynamicEventHandlerInstanceSymbol event, BluePrintCPP bluePrint){
        String name = "free_"+String.join("_", names);
        Method method = null;
        if(bluePrint.getMethod(name).isPresent()){
            Log.info("Extend free method: "+name+"(...)", "EventConverter");
            method = bluePrint.getMethod(name).get();
        }else{
            Log.info("Create free method: "+name+"(...)", "EventConverter");
            method = new Method(name, "void");
            for(String n : names){
                Variable v = new Variable();
                v.setName(n+"_Idx");
                v.setTypeNameTargetLanguage("int");
                method.addParameter(v);
            }
        }

        method.addInstruction(new TargetCodeInstruction("for(long i = __event_connects_"+convertName(event.getName())+".size()-1; i >= 0; --i){\n"));
        method.addInstruction(new TargetCodeInstruction("int* _connected_idxs = __event_connects_"+convertName(event.getName())+".at(i);\n"));
        method.addInstruction(new TargetCodeInstruction("if( "));

        for(int i = 0; i < names.size(); ++i){
            method.addInstruction(new TargetCodeInstruction(
                    "(_connected_idxs["+i+"] == "+names.get(i)+"_Idx)"
            ));
            if(i < names.size()-1){
                method.addInstruction(new TargetCodeInstruction(" && "));
            }
        }
        method.addInstruction(new TargetCodeInstruction(" ){\n"));

        bluePrint.addMethod(method);
        return method;
    }

    protected static void generateEndOfFreeMethod(EMADynamicEventHandlerInstanceSymbol event, Method free, EMAComponentInstanceSymbol componentSymbol){

        String name = String.format(FREE_CONNECTIONTRACARRAY, convertName(event.getName()));
        free.addInstruction(new TargetCodeInstruction(
                name+".erase("+name+".begin()+i);\n"
        ));
        for(String s : resetConnectedArray){
            free.addInstruction(new TargetCodeInstruction(s));
        }
        free.addInstruction(new TargetCodeInstruction("free(_connected_idxs);\n"));

//        free.addInstruction(new TargetCodeInstruction("//----------------------------------------------------------------------------\n"));
        List<String> names = new ArrayList<>();
        event.getCondition().getConnectPortNames(names);
        for(EMADynamicEventHandlerInstanceSymbol freeEvent : getFreeEvents(componentSymbol)){
            List<String> freeNames = new ArrayList<>();
            freeEvent.getCondition().getFreePortNames(freeNames);
            freeNames.removeAll(names);

            if(freeNames.isEmpty()){
                // free event
                free.addInstruction(new TargetCodeInstruction("if("+EventConnectInstructionCPP.getEventNameCPP(freeEvent.getName())+"){\n"));
                generateConnectsForFreeEventBody(freeEvent, free);
                free.addInstruction(new TargetCodeInstruction("}\n"));
            }
        }
//        free.addInstruction(new TargetCodeInstruction("//----------------------------------------------------------------------------\n"));

        free.addInstruction(new TargetCodeInstruction("}\n}\n"));
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

    protected static void generateHandleConnectRequestsOfQueues(List<String> names, Method body, Method free){
        for(String name : names){
            body.addInstruction(new TargetCodeInstruction(
                    String.format(THISCOMPONENTPORTREQUEST,
                            name, name, name, free_method_index_counter, name)));

            free.addInstruction(new TargetCodeInstruction(String.format(FREE_THISCOMPONENTPORTREQUEST,
                name, free_method_index_counter
            )));

            resetConnectedArray.add(String.format(FREE_THISCOMPONENTPORTREQUEST_CONNECTED_ARRAY, name,name));

            free_method_index_counter++;
        }
    }

    protected static void generateHandleConnectRequestsOfDynamicInstances(List<String> instancenames, Method body, EMAComponentInstanceSymbol componentSymbol, Method free){
        for (String inst : instancenames){
            int count = 1;
            while (componentSymbol.getSubComponent(inst+"["+count+"]").isPresent()){
                ++count;
            }
            body.addInstruction(new TargetCodeInstruction(String.format(DYNINSTANCECONNECT,
                    inst, count-1, inst, inst, free_method_index_counter, inst
                    )));

            free.addInstruction(new TargetCodeInstruction(String.format(FREE_DYNINSTANCECONNECT,
                    inst, free_method_index_counter
            )));

            resetConnectedArray.add(String.format(FREE_DYNINSTANCECONNECT_CONNECTED_ARRAY, inst,inst));

            free_method_index_counter++;
        }
    }

    protected static void generateHandleConnectRequestInInstances(Map<String, List<String>> newPorts, Method body, Method free){
        for (Map.Entry<String,List<String>> entry : newPorts.entrySet()){
            Collections.sort(entry.getValue());
            String inst = entry.getKey()+".connect_"+String.join("_", entry.getValue())+"(";
            String freeInst = entry.getKey()+".free_"+String.join("_", entry.getValue())+"(";

            String connectIdxs = "";
//            for(String port : entry.getValue()){
            for(int i = 0; i < entry.getValue().size(); ++i){

                body.addInstruction(new TargetCodeInstruction(String.format(
                        DYNPORTIDININSTANCEINIT, convertName(entry.getKey()), entry.getValue().get(i)
                )));

                inst = inst + "&"+String.format(DYNPORTIDININSTANCE, convertName(entry.getKey()), entry.getValue().get(i));
                if(i < entry.getValue().size()-1){
                    inst += ", ";
                }

                free.addInstruction(new TargetCodeInstruction(String.format(FREE_DYNPORTIDININSTANCE,
                        convertName(entry.getKey()), entry.getValue().get(i), free_method_index_counter)));

                freeInst += String.format(DYNPORTIDININSTANCE, convertName(entry.getKey()), entry.getValue().get(i));
                if(i < entry.getValue().size()-1){
                    freeInst += ", ";
                }

                connectIdxs += "_connected_idxs["+free_method_index_counter+"] = "+String.format(DYNPORTIDININSTANCE, convertName(entry.getKey()), entry.getValue().get(i))+";";

                free_method_index_counter++;


            }

            free.addInstruction(new TargetCodeInstruction(
                freeInst+");\n"
            ));

            body.addInstruction(new TargetCodeInstruction(String.format(
                    "if(!"+inst+")){ free(_connected_idxs); return ;}\n"
            )));
            body.addInstruction(new TargetCodeInstruction(connectIdxs+"\n"));

        }
    }

    protected static void generateConnects(EMADynamicEventHandlerInstanceSymbol event, Method body, BluePrintCPP bluePrint, Method executeMethod, Method free){
        for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){

            Optional<String> before = generateConnectsBefore(connector);
            String sourceName = generateConnectsSource(connector);
            String targetName = generateConnectsTarget(connector);
            EMAPortInstanceSymbol target = connector.getTargetPort();

            Optional<VariableType> vt = TypeConverter.getVariableTypeForMontiCarTypeName(target.getTypeReference().getName());
            generateEventDynamicConnectVector(vt.get().getTypeNameTargetLanguage(), bluePrint);

            executeMethod.addInstruction(new ExecuteDynamicConnects(before));

            if(!before.isPresent()){
                before = Optional.of("NULL");
            }else{
                before = Optional.of("&"+before.get());
            }
            body.addInstruction(new TargetCodeInstruction(String.format(
                    "__dynamic_%s_connect.push_back({%s, &(%s), &(%s)});\n", vt.get().getTypeNameTargetLanguage(), before.get(), sourceName, targetName
            )));

            free.addInstruction(new TargetCodeInstruction(String.format("dynamicconnect_remove(&__dynamic_%s_connect, %s, &(%s), &(%s));\n",
                    vt.get().getTypeNameTargetLanguage(), before.get(), sourceName, targetName
                    )));
        }
    }

    protected static String generateConnectsSource(EMADynamicConnectorInstanceSymbol connector){
        String sourceName = connector.getSource();
        if(connector.isDynamicSourceNewPort()) {
            sourceName = generateConnectNameForNewPort(sourceName, connector.getSourceComponentName(), connector.getSourcePortName());
        }else if(connector.isDynamicSourceNewComponent()){
            sourceName = generateConnectNameForNewInstance(connector.getSourceComponentName().get(), connector.getSourcePortName());
        }else{
            sourceName = GeneralHelperMethods.getTargetLanguageVariableInstanceName(sourceName);
        }
        return sourceName;
    }

    protected static String generateConnectsTarget(EMADynamicConnectorInstanceSymbol connector){
        String targetName = connector.getTarget();
        if(connector.isDynamicTargetNewPort()) {
            targetName = generateConnectNameForNewPort(targetName, connector.getTargetComponentName(), connector.getTargetPortName());
//            before = connector.getTargetComponentName();
//            if (before.isPresent()) {
//                before = Optional.of(GeneralHelperMethods.getTargetLanguageVariableInstanceName(before.get()));
//            }
        }else if(connector.isDynamicTargetNewComponent()){
            targetName = generateConnectNameForNewInstance(connector.getTargetComponentName().get(), connector.getTargetPortName());
//            String inst = EMAPortSymbol.getNameWithoutArrayBracketPart(connector.getTargetComponentName().get());
//            before = Optional.of(String.format("%s["+DYNINSTANCEID+"]", inst, inst));
        }else{
            targetName = GeneralHelperMethods.getTargetLanguageVariableInstanceName(targetName);
//            before = connector.getTargetComponentName();
//            if(before.isPresent()){
//                before = Optional.of(GeneralHelperMethods.getTargetLanguageVariableInstanceName(before.get()));
//            }
        }
        return targetName;
    }

    protected static Optional<String> generateConnectsBefore(EMADynamicConnectorInstanceSymbol connector){
        Optional<String> before = Optional.empty();
//        String targetName = connector.getTarget();
        if(connector.isDynamicTargetNewPort()) {
//            targetName = generateConnectNameForNewPort(targetName, connector.getTargetComponentName(), connector.getTargetPortName());
            before = connector.getTargetComponentName();
            if (before.isPresent()) {
                before = Optional.of(GeneralHelperMethods.getTargetLanguageVariableInstanceName(before.get()));
            }
        }else if(connector.isDynamicTargetNewComponent()){
//            targetName = generateConnectNameForNewInstance(connector.getTargetComponentName().get(), connector.getTargetPortName());
            String inst = EMAPortSymbol.getNameWithoutArrayBracketPart(connector.getTargetComponentName().get());
            before = Optional.of(String.format("%s["+DYNINSTANCEID+"]", inst, inst));
        }else{
//            targetName = GeneralHelperMethods.getTargetLanguageVariableInstanceName(targetName);
            before = connector.getTargetComponentName();
            if(before.isPresent()){
                before = Optional.of(GeneralHelperMethods.getTargetLanguageVariableInstanceName(before.get()));
            }
        }
        return before;
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

    protected static String generateConnectNameForNewInstance(String componentName, String portName){

        String inst = EMAPortSymbol.getNameWithoutArrayBracketPart(componentName);

        return String.format("%s["+DYNINSTANCEID+"].%s", inst, inst, GeneralHelperMethods.getTargetLanguageVariableInstanceName(portName));

    }


    protected static void generateConnectsForFreeEventBody(EMADynamicEventHandlerInstanceSymbol event, Method free){

        for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){
            String sourceName = generateConnectsSource(connector);
            String targetName = generateConnectsTarget(connector);
            free.addInstruction(new TargetCodeInstruction(targetName+" = "+sourceName+";\n"));
        }
    }

    protected static String convertName(String name){
        return name.replace("[", "_").replace("]", "_");
    }

    protected static Collection<EMADynamicEventHandlerInstanceSymbol> getFreeEvents(EMAComponentInstanceSymbol component){
        if(component instanceof EMADynamicComponentInstanceSymbol) {
            return ((EMADynamicComponentInstanceSymbol)component).getEventHandlers().stream().filter(e -> e.isDynamicPortFreeEvent()).collect(Collectors.toList());
        }
        return new ArrayList<>();
    }
}
